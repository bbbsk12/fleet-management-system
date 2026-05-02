#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#include <tf2/LinearMath/Quaternion.hpp>

namespace fleet_manager
{
// ==================== 冲突检测与解决（核心简化） ====================

void FleetManagerNode::conflict_check_timer_callback()
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  const auto now = this->now();
  // 核心策略：阻塞先观察，持续一段时间后再升级为 hold/避让，避免岔口瞬态误触发。
  constexpr double kBlockEscalationDelaySec = 2.0;

  // 1) 更新所有在线机器人离散位置
  //    offline 机器人的锁保留（ghost guard，防止碰撞），
  //    此处不再更新它们的位置
  for (auto & [robot_id, st] : robot_statuses_) {
    if (st.connection_status != "online") continue;
    // 检查位姿有效性（全零位姿说明无定位，跳过更新避免误判位置）
    if (std::abs(st.current_pose.position.x) < 1e-6 &&
        std::abs(st.current_pose.position.y) < 1e-6) {
      // 关键修复：无效位姿时不能保留历史占位锁，否则会形成“机器人看似空闲
      // 但 check_can_enter 长期被旧锁阻塞”的假死循环。
      occupancy_manager_->release_locks(robot_id);
      continue;
    }
    auto loc = occupancy_manager_->update_robot_location(
      robot_id, st.current_pose, waypoint_acceptance_radius_, traffic_segment_lateral_max_);

    // 更新 RobotStatus 中的离散位置字段
    if (loc.type == LocationType::WAYPOINT) {
      st.location_type = "waypoint";
      st.current_waypoint = loc.waypoint_id;
      st.current_segment = "";
    } else if (loc.type == LocationType::SEGMENT) {
      st.location_type = "segment";
      st.current_waypoint = "";
      st.current_segment = loc.segment_from + "->" + loc.segment_to;
    } else {
      st.location_type = "unknown";
    }
  }

  // 2) 同航点/航段去重
  auto overlaps = occupancy_manager_->resolve_overlaps();
  for (auto & [robot_id, avoid_wp] : overlaps) {
    if (avoid_wp.empty()) continue;
    PersistLogger::log_warn(
      "overlap", robot_id, "",
      "overlap detected, suggest avoidance_wp=" + avoid_wp,
      __FILE__, __LINE__, __func__);
    if (robot_yieldable_as_idle_blocker(robot_id)) {
      request_move_idle_robot_to_avoidance(robot_id, avoid_wp, /*force=*/false);
    }
  }

  // 2.5) 清理不再执行内部任务的机器人的避让代数
  //       防止代数累积导致机器人永远无法被避让
  for (auto it = robot_avoidance_generation_.begin(); it != robot_avoidance_generation_.end(); ) {
    auto nav = get_robot_nav_info(it->first);
    if (!nav || nav->current_task_id.empty() || !is_internal_task_id(nav->current_task_id)) {
      // 不再执行内部任务 → 重置代数
      it = robot_avoidance_generation_.erase(it);
    } else {
      ++it;
    }
  }

  // 3) 对每个正在导航的机器人，检查下一跳是否被阻塞
  //    按汇入排队优先级排序：优先级低的（0=最高）先检查，先获取锁
  std::set<std::string> currently_blocked;
  {
    // 收集需要检查的机器人，按优先级排序
    std::vector<std::pair<int, std::string>> sorted_robots;
    for (auto & [robot_id, nav_info] : robot_nav_info_) {
      if (!nav_info || !nav_info->has_active_goal) continue;
      if (is_robot_hard_held(robot_id)) continue;
      if (nav_info->route_waypoints.empty()) continue;
      int pri = 0;
      auto pri_it = robot_merge_priority_.find(robot_id);
      if (pri_it != robot_merge_priority_.end()) pri = pri_it->second;
      sorted_robots.push_back({pri, robot_id});
    }
    std::stable_sort(sorted_robots.begin(), sorted_robots.end());

    for (const auto & [pri, robot_id] : sorted_robots) {
      auto nav_it = robot_nav_info_.find(robot_id);
      if (nav_it == robot_nav_info_.end()) continue;
      auto & nav_info = nav_it->second;
    if (!nav_info || !nav_info->has_active_goal) continue;
    if (is_robot_hard_held(robot_id)) continue;
    if (nav_info->route_waypoints.empty()) continue;

    // 找当前位置和下一个目标
    std::string current_wp;
    auto st_it = robot_statuses_.find(robot_id);
    if (st_it != robot_statuses_.end() && st_it->second.location_type == "waypoint") {
      current_wp = st_it->second.current_waypoint;
    }
    if (current_wp.empty() && nav_info->current_waypoint_index < nav_info->route_waypoints.size()) {
      current_wp = nav_info->route_waypoints[std::min(nav_info->current_waypoint_index,
                                                       nav_info->route_waypoints.size() - 1)];
    }

    // 检查从当前位置到路径上下一个航点是否可进入
    for (size_t i = nav_info->current_waypoint_index; i + 1 < nav_info->route_waypoints.size(); ++i) {
      const auto & from_wp = nav_info->route_waypoints[i];
      const auto & to_wp = nav_info->route_waypoints[i + 1];
      const auto enter_check = occupancy_manager_->check_can_enter_detailed(robot_id, from_wp, to_wp);
      const std::string blocker = enter_check.blocker;
      if (!blocker.empty()) {
        PersistLogger::log_info(
          "conflict.blocked_reason", robot_id, nav_info->current_task_id,
          "from=" + from_wp + " to=" + to_wp +
          " blocker=" + blocker + " reason=" +
          OccupancyManager::enter_block_reason_to_string(enter_check.reason),
          __FILE__, __LINE__, __func__);
        currently_blocked.insert(robot_id);
        robot_blocked_by_[robot_id] = blocker;
        if (robot_blocked_since_.find(robot_id) == robot_blocked_since_.end()) {
          robot_blocked_since_[robot_id] = now;
        }
        const auto since_it = robot_blocked_since_.find(robot_id);
        const double blocked_sec = (since_it != robot_blocked_since_.end()) ?
          (now - since_it->second).seconds() : 0.0;

        // Unified offline-blocker policy: user task times out, internal task keeps normal flow.
        {
          auto blocker_st_it = robot_statuses_.find(blocker);
          const bool blocker_offline = (blocker_st_it == robot_statuses_.end() ||
                                        blocker_st_it->second.connection_status != "online");
          if (blocker_offline && !is_internal_task_id(nav_info->current_task_id)) {
            const double offline_fail_timeout = 5.0;
            if (fail_task_if_offline_blocker_timeout(
                  robot_id, nav_info->current_task_id, blocker, blocked_sec, offline_fail_timeout))
            {
              break;  // task finished
            }
            continue;
          }
        }

        // 规则③：如果阻塞者是可让行的，调度避让
        if (robot_yieldable_as_idle_blocker(blocker)) {
          if (blocked_sec < kBlockEscalationDelaySec) {
            // 先观测，等待阻塞持续再升级处置
            continue;
          }
          auto cooldown_it = robot_avoidance_cooldown_until_.find(blocker);
          if (cooldown_it == robot_avoidance_cooldown_until_.end() || now >= cooldown_it->second) {
            std::vector<std::string> exclude(nav_info->route_waypoints.begin(), nav_info->route_waypoints.end());
            const std::string dest = nav_info->route_waypoints.empty() ? to_wp : nav_info->route_waypoints.back();
            (void)handle_blocking_with_unified_policy(
              robot_id, nav_info->current_task_id, from_wp, to_wp, dest, exclude, blocker,
              /*use_dispatch_requests=*/true, "conflict");
          }
        } else if (!is_robot_hard_held(robot_id)) {
          if (blocked_sec < kBlockEscalationDelaySec) {
            continue;
          }
          // 死胡同出车优化：阻塞者不可让行（正在执行任务），但请求者被困在死胡同
          // 检查 from_wp 是否只有一个邻居（即 from_wp 是死胡同末端）
          // 此时即使阻塞者有任务，也需要让它暂时让路
          {
            auto adj_from = traffic_manager_->get_adjacency_map().find(from_wp);
            bool is_dead_end = (adj_from != traffic_manager_->get_adjacency_map().end() &&
                                adj_from->second.size() <= 1);
            // 也检查请求者被阻塞了较长时间（> 避让冷却时间），说明确实无法通过正常等待解决
            auto blocked_since_it = robot_blocked_since_.find(robot_id);
            bool blocked_long_enough = false;
            if (blocked_since_it != robot_blocked_since_.end()) {
              blocked_long_enough = (now - blocked_since_it->second).seconds() >= avoidance_cooldown_sec_;
            }

            if (is_dead_end && blocked_long_enough) {
              std::vector<std::string> exclude(
                nav_info->route_waypoints.begin(), nav_info->route_waypoints.end());
              (void)try_force_yield_dead_end_blocker(
                robot_id, nav_info->current_task_id, from_wp, to_wp, blocker, exclude, now);
            }
          }
        }
        break;
      }
    }
  }  // end sorted_robots loop
  }  // end priority-sorted block

  // 清理不再阻塞的记录
  for (auto it = robot_blocked_by_.begin(); it != robot_blocked_by_.end();) {
    if (currently_blocked.find(it->first) == currently_blocked.end()) {
      robot_blocked_since_.erase(it->first);
      it = robot_blocked_by_.erase(it);
    } else {
      ++it;
    }
  }

  // 3.5) 多车汇入排队：分析同一目标航点的多个等待者，按等待时间分配优先通行权
  // 当多台机器人同时想进入同一航点时，等待最久的优先级最高
  // 超时后强制提升优先级，防止饥饿
  {
    // 统计每个目标航点的等待者列表
    std::map<std::string, std::vector<std::pair<rclcpp::Time, std::string>>> merge_queue;
    for (const auto & [rid, blocker] : robot_blocked_by_) {
      auto nav = get_robot_nav_info(rid);
      if (!nav || nav->route_waypoints.size() < 2) continue;
      if (nav->current_waypoint_index + 1 >= nav->route_waypoints.size()) continue;
      const std::string target_wp = nav->route_waypoints[nav->current_waypoint_index + 1];
      auto since_it = robot_blocked_since_.find(rid);
      rclcpp::Time since = (since_it != robot_blocked_since_.end()) ? since_it->second : now;
      merge_queue[target_wp].push_back({since, rid});
    }

    // 对每个目标航点，按等待时间排序（等待最久的优先）
    for (auto & [target_wp, waiters] : merge_queue) {
      if (waiters.size() < 2) continue;  // 只有1个等待者不需要排队
      std::sort(waiters.begin(), waiters.end());

      // 更新优先级
      int priority = 0;
      for (auto & [since, rid] : waiters) {
        robot_merge_priority_[rid] = priority++;
        robot_merge_queue_since_[rid] = since;

        // 超时提升优先级：等待超过 merge_queue_timeout_sec_ 的机器人获得最高优先级
        if ((now - since).seconds() >= merge_queue_timeout_sec_ && robot_merge_priority_[rid] > 0) {
          PersistLogger::log_warn(
            "merge.queue_timeout", rid, "",
            "waiting at " + target_wp + " for " +
            std::to_string(static_cast<int>((now - since).seconds())) + "s, promoting priority",
            __FILE__, __LINE__, __func__);
          robot_merge_priority_[rid] = 0;  // 最高优先级
        }
      }
    }

    // 清理不再阻塞的机器人的排队记录
    for (auto it = robot_merge_queue_since_.begin(); it != robot_merge_queue_since_.end(); ) {
      if (currently_blocked.find(it->first) == currently_blocked.end()) {
        robot_merge_priority_.erase(it->first);
        it = robot_merge_queue_since_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // 4) 死锁环检测（改进版：支持间接资源死锁）
  // 构建阻塞图：不仅从 robot_blocked_by_，还检查 hold 等待关系
  std::map<std::string, std::string> block_graph = robot_blocked_by_;
  for (const auto & [rid, hctx] : hold_contexts_) {
    if (!hctx.active || hctx.wait_for_robot.empty()) continue;
    if (block_graph.find(rid) == block_graph.end()) {
      block_graph[rid] = hctx.wait_for_robot;
    }
  }

  std::vector<std::string> cycle;
  for (const auto & [rid, _] : block_graph) {
    std::map<std::string, size_t> seen_index;
    std::vector<std::string> chain;
    std::string cur = rid;
    while (!cur.empty()) {
      if (seen_index.count(cur)) {
        cycle.assign(chain.begin() + static_cast<long>(seen_index[cur]), chain.end());
        break;
      }
      seen_index[cur] = chain.size();
      chain.push_back(cur);
      auto nxt = block_graph.find(cur);
      if (nxt == block_graph.end()) break;
      cur = nxt->second;
    }
    if (cycle.size() >= 2) break;
  }

  if (cycle.size() >= 2) {
    rclcpp::Time oldest = now;
    for (const auto & rid : cycle) {
      auto it = robot_blocked_since_.find(rid);
      if (it != robot_blocked_since_.end() && it->second < oldest) oldest = it->second;
    }
    if ((now - oldest).seconds() >= deadlock_wait_sec_) {
      const auto decision = decide_deadlock_break_resolution(cycle, now, "conflict");
      if (decision.type == BlockingDecisionType::DEADLOCK_YIELD_SELF) {
        (void)apply_blocking_resolution(
          decision, decision.actor_robot_id, /*task_id=*/"", /*from_wp=*/"", /*blocked_wp=*/"",
          /*destination_wp=*/"", /*blocker_robot_id=*/"", /*use_dispatch_requests=*/true, "conflict");
      }
    }
  }

  // Defer side-effects to single-writer dispatch.
  request_process_hold_state_machine();
  request_publish_traffic_fleet_status();
}

}  // namespace fleet_manager
