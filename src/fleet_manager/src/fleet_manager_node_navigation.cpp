#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>

namespace fleet_manager
{

// ============================================================================
// 常量
// ============================================================================

namespace
{
constexpr int    kMaxChainDepth        = 5;
constexpr double kChainTotalTimeout    = 180.0;
constexpr double kChainStepTimeout     = 8.0;
constexpr int    kMaxChainStepRetries  = 2;
constexpr const char * kChainTaskPrefix = "chain_retreat_";
}  // namespace

// ============================================================================
// 导航入口 — 从当前位姿到目标航点的完整导航启动
// ============================================================================

bool FleetManagerNode::start_navigation(
  const std::string & robot_id,
  const std::string & target_wp,
  const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);
  if (!ni) { scheduler_->fail_task(task_id, "no nav client"); return false; }

  const auto now = this->now();

  // 底盘被占用或正在退避中 → 延迟
  if (ni->has_active_goal || !ni->route.empty() || !ni->current_task_id.empty()) {
    scheduler_->mark_task_waiting(task_id);
    return false;
  }
  if (ni->retry_after.nanoseconds() > 0 && now < ni->retry_after) {
    scheduler_->mark_task_waiting(task_id);
    return false;
  }

  auto st = robots_.find(robot_id);
  if (st == robots_.end()) { scheduler_->fail_task(task_id, "robot not found"); return false; }

  // 已在目标航点 → 直接完成
  auto target_pose = traffic_->get_waypoint_pose(target_wp);
  double dx = st->second.current_pose.position.x - target_pose.position.x;
  double dy = st->second.current_pose.position.y - target_pose.position.y;
  if (std::hypot(dx, dy) <= waypoint_radius_) {
    on_nav_succeeded(robot_id, task_id);
    return true;
  }

  if (!is_internal_task_id(task_id)) {
    std::string target_blocker = occupancy_->waypoint_blocker(robot_id, target_wp);
    if (!target_blocker.empty()) {
      scheduler_->mark_task_waiting(task_id);
      clear_task_wait_condition(task_id);
      task_waits_[task_id] = {TaskWaitState::WAIT_TARGET_CLEAR, robot_id, task_id,
        target_blocker, std::string{}, std::string{}, target_wp, ni->retry_count};
      waiting_for_[target_blocker].insert(robot_id);
      ni->retry_after = rclcpp::Time{};
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
      if (!ti.task_id.empty()) task_pub_->publish(ti);
      PersistLogger::log_info("nav.target_blocked_wait", robot_id, task_id,
        "target " + target_wp + " blocked by " + target_blocker,
        __FILE__, __LINE__, __func__);
      return false;
    }
  }

  // 找到最近的航点作为路径起点
  std::string start_wp;
  double min_d = std::numeric_limits<double>::max();
  for (const auto & [id, pose] : traffic_->get_all_waypoint_poses()) {
    double d = std::hypot(
      st->second.current_pose.position.x - pose.position.x,
      st->second.current_pose.position.y - pose.position.y);
    if (d < min_d) { min_d = d; start_wp = id; }
  }

  // 验证底盘拥有起始航点的 zone(或被占用但足够近)
  if (!start_wp.empty()) {
    std::string holder = occupancy_->get_zone_holder(start_wp);
    if (!holder.empty() && holder != robot_id) {
      ni->retry_after = now + rclcpp::Duration::from_seconds(0.5);
      scheduler_->mark_task_waiting(task_id);
      return false;
    }
    if (holder.empty()) {
      auto start_pose = traffic_->get_waypoint_pose(start_wp);
      double d2 = std::hypot(
        st->second.current_pose.position.x - start_pose.position.x,
        st->second.current_pose.position.y - start_pose.position.y);
      if (d2 > waypoint_radius_ * 2.5) {
        ni->retry_after = now + rclcpp::Duration::from_seconds(0.5);
        scheduler_->mark_task_waiting(task_id);
        return false;
      }
    }
  }

  // 占用感知寻路(优先避开已占用的航点/航段), 无路径时回退普通 BFS
  std::vector<std::string> path;
  if (!start_wp.empty() && start_wp != target_wp) {
    auto occ_zones = occupancy_->get_occupied_zones();
    std::set<std::string> occ_edges;
    for (const auto & [wp, conns] : traffic_->get_adjacency_map()) {
      if (occ_zones.count(wp)) {
        for (const auto & nb : conns)
          occ_edges.insert(wp < nb ? (wp + "|" + nb) : (nb + "|" + wp));
      }
    }
    path = traffic_->find_path_weighted(start_wp, target_wp, occ_edges, occ_zones);
    if (path.empty())
      path = traffic_->find_path(start_wp, target_wp);
  }
  if (path.empty()) path.push_back(target_wp);

  // 去重连续相同航点
  std::vector<std::string> clean;
  for (const auto & w : path) {
    if (clean.empty() || clean.back() != w) clean.push_back(w);
  }
  path = clean;
  if (path.empty()) path.push_back(target_wp);

  if (!is_internal_task_id(task_id)) {
    std::string conflict_task;
    std::string conflict_resource;
    auto route_blocker = find_active_route_conflict(robot_id, task_id, path,
      conflict_task, conflict_resource, true);
    if (!route_blocker.empty()) {
      scheduler_->mark_task_waiting(task_id);
      clear_task_wait_condition(task_id);
      task_waits_[task_id] = {TaskWaitState::WAIT_ROUTE_CLEAR, robot_id, task_id,
        route_blocker, path.empty() ? std::string{} : path.front(),
        path.size() > 1 ? path[1] : std::string{}, target_wp, ni->retry_count};
      waiting_for_[route_blocker].insert(robot_id);
      ni->retry_after = rclcpp::Time{};
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
      if (!ti.task_id.empty()) task_pub_->publish(ti);
      PersistLogger::log_info("nav.route_active_wait", robot_id, task_id,
        "route " + join_waypoints(path) + " waits for blocking task " + conflict_task +
        " on " + route_blocker + " resource=" + conflict_resource,
        __FILE__, __LINE__, __func__);
      return false;
    }
  }

  // 检查首跳是否被阻塞。离线阻塞者立即失败；在线阻塞者交给 navigate_to_next_waypoint 做协调。
  if (path.size() >= 2) {
    std::string blocker = occupancy_->can_enter(robot_id, path[0], path[1]);
    if (!blocker.empty()) {
      auto bs = robots_.find(blocker);
      if (bs == robots_.end() || bs->second.connection_status != "online") {
        scheduler_->fail_task(task_id, "blocked by offline robot " + blocker);
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
        task_pub_->publish(ti);
        return false;
      }
      PersistLogger::log_info("nav.first_hop_blocked_online", robot_id, task_id,
        "first hop " + path[0] + "->" + path[1] + " blocked by online " + blocker +
        ", delegating coordination to navigate_to_next_waypoint",
        __FILE__, __LINE__, __func__);
    }
  }

  if (path.size() <= 1) {
    std::string wp = path[0];
    std::string blocker = occupancy_->waypoint_blocker(robot_id, wp);
    if (!blocker.empty()) {
      ni->retry_count++;
      double jitter = 0.7 + 0.6 * (static_cast<double>(std::hash<std::string>{}(robot_id) % 1000) / 1000.0);
      double backoff = jitter * retry_base_ * std::pow(1.5, std::min(ni->retry_count, retry_max_));
      ni->retry_after = now + rclcpp::Duration::from_seconds(backoff);
      scheduler_->mark_task_waiting(task_id);
      return false;
    }
  }

  if (ni->has_active_goal || ni->goal_handle || ni->align_goal_handle || ni->aligning_before_nav) {
    cancel_goals(ni);
  } else {
    ni->nav_seq++;
  }

  ni->route       = path;
  ni->route_index = 0;
  ni->route_alignment_done = false;
  ni->current_task_id = task_id;
  ni->retry_count = 0;
  ni->retry_after = rclcpp::Time{};
  clear_task_wait_condition(task_id);

  RCLCPP_INFO(this->get_logger(),
    "nav.start task=%s robot=%s path=%zu waypoints: %s",
    task_id.c_str(), robot_id.c_str(), path.size(), join_waypoints(path).c_str());
  PersistLogger::log_info("nav.start", robot_id, task_id,
    "path=" + join_waypoints(path),
    __FILE__, __LINE__, __func__);

  scheduler_->mark_task_navigating(task_id);
  fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
  if (!ti.task_id.empty()) task_pub_->publish(ti);
  navigate_to_next_waypoint(robot_id);
  return true;
}

// ============================================================================
// 逐航点导航 — 自动跳过已到达的航点，向路径上的下一个航点发送 goal
// ============================================================================

void FleetManagerNode::navigate_to_next_waypoint(const std::string & robot_id)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);
  if (!ni || ni->route.empty()) return;

  if (ni->has_active_goal) return;  // 已有活跃 goal
  if (ni->current_task_id.empty()) return;

  size_t n = ni->route.size();
  if (ni->route_index >= n) return;

  auto st = robots_.find(robot_id);

  // 根据底盘实际位姿跳过已物理到达的航点
  size_t target = ni->route_index;
  if (st != robots_.end()) {
    if (target == 0 && n > 1) {
      const std::string & route_start = ni->route.front();
      bool at_route_start = st->second.current_waypoint == route_start;
      auto loc = occupancy_->get_location(robot_id);
      if (!at_route_start && loc.type == LocationType::WAYPOINT) {
        at_route_start = loc.waypoint_id == route_start;
      }
      if (!at_route_start && loc.type == LocationType::SEGMENT) {
        at_route_start = loc.segment_from == route_start || loc.segment_to == route_start;
      }
      if (!at_route_start) {
        const auto nearest = traffic_->find_nearest_waypoint(st->second.current_pose);
        if (nearest == route_start) {
          auto start_pose = traffic_->get_waypoint_pose(route_start);
          double d = std::hypot(
            st->second.current_pose.position.x - start_pose.position.x,
            st->second.current_pose.position.y - start_pose.position.y);
          at_route_start = d <= waypoint_radius_ * 2.5;
        }
      }
      if (at_route_start) {
        target = 1;
        PersistLogger::log_info("nav.skip_route_start", robot_id, ni->current_task_id,
          "skipping route start " + route_start + " toward " + ni->route[target],
          __FILE__, __LINE__, __func__);
      }
    }
    while (target < n) {
      auto wp_pose = traffic_->get_waypoint_pose(ni->route[target]);
      double d = std::hypot(
        st->second.current_pose.position.x - wp_pose.position.x,
        st->second.current_pose.position.y - wp_pose.position.y);
      if (d <= waypoint_radius_) target++;
      else break;
    }
  }
  ni->route_index = target;
  if (ni->route_index >= n) {
    std::string task_id = ni->current_task_id;
    if (chain_plan_.active && task_id.rfind(kChainTaskPrefix, 0) == 0) {
      on_chain_step_complete(robot_id, true);
    } else {
      on_nav_succeeded(robot_id, task_id);
    }
    return;
  }

  std::string wp     = ni->route[target];
  bool is_final      = (target == n - 1);

  // 检查并预留当前跳
  if (target > 0 && target < n) {
    std::string from = ni->route[target - 1];
    std::string blocker = occupancy_->can_enter(robot_id, from, wp);
    if (blocker.empty()) {
      blocker = physical_waypoint_blocker(robot_id, wp);
    }
    if (!blocker.empty()) {
      PersistLogger::log_info("nav.hop_blocked", robot_id, ni->current_task_id,
        "from=" + from + " to=" + wp + " blocker=" + blocker,
        __FILE__, __LINE__, __func__);
      occupancy_->release_reservations(robot_id);

      // #3 reroute 自愈: 同一 hop 反复 blocked → 尝试绕过 blocker 占用的 wp
      {
        const std::string blk_key = from + "->" + wp + ":" + blocker;
        if (ni->hop_block_key == blk_key) {
          ni->hop_block_count++;
        } else {
          ni->hop_block_key = blk_key;
          ni->hop_block_count = 1;
        }
        if (ni->hop_block_count >= 5 && !is_final &&
            !ni->route.empty() &&
            ni->current_task_id.rfind(kChainTaskPrefix, 0) != 0 &&
            !chain_plan_.active)
        {
          const std::string final_wp = ni->route.back();
          // 硬避让:完全绕开 blocker 占用的 wp。
          // 软成本(find_path_weighted)在拓扑被迫经过 wp 时仍返回经过路径,
          // 导致 reroute_ok 永远 false。
          std::set<std::string> avoid_wps{wp};
          auto alt = traffic_->find_path_avoiding(from, final_wp, avoid_wps);
          const bool reroute_ok = alt.size() >= 2 &&
            std::find(alt.begin(), alt.end(), wp) == alt.end();
          if (reroute_ok) {
            PersistLogger::log_info("nav.hop_reroute", robot_id, ni->current_task_id,
              "rerouting around " + wp + " (blocked " +
              std::to_string(ni->hop_block_count) + "x by " + blocker +
              "), new path=" + join_waypoints(alt),
              __FILE__, __LINE__, __func__);
            ni->route = alt;
            ni->route_index = 0;
            ni->route_alignment_done = false;
            ni->retry_count = 0;
            ni->retry_after = rclcpp::Time{};
            ni->hop_block_key.clear();
            ni->hop_block_count = 0;
            // 清除等待: 新路径不再依赖原 blocker
            for (auto & [_, ws] : waiting_for_) ws.erase(robot_id);
            return;  // 下一 tick 用新 route 重新进入本函数
          }
        }
      }

      std::string target_blocker;
      if (is_final && !is_internal_task_id(ni->current_task_id)) {
        target_blocker = occupancy_->waypoint_blocker(robot_id, wp);
        if (target_blocker.empty()) {
          target_blocker = physical_waypoint_blocker(robot_id, wp);
        }
      }
      if (!target_blocker.empty()) {
        std::string tid = ni->current_task_id;
        scheduler_->mark_task_waiting(tid);
        clear_task_wait_condition(tid);
        task_waits_[tid] = {TaskWaitState::WAIT_TARGET_CLEAR, robot_id, tid,
          target_blocker, std::string{}, std::string{}, wp, ni->retry_count};
        waiting_for_[target_blocker].insert(robot_id);
        occupancy_->release_reservations(robot_id);
        ni->has_active_goal = false;
        ni->current_task_id.clear();
        ni->route.clear();
        ni->route_index = 0;
        ni->retry_count = 0;
        ni->retry_after = rclcpp::Time{};
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
        if (!ti.task_id.empty()) task_pub_->publish(ti);
        PersistLogger::log_info("nav.final_target_blocked_wait", robot_id, tid,
          "target " + wp + " blocked by " + target_blocker,
          __FILE__, __LINE__, __func__);
        return;
      }

      if (chain_plan_.active && ni->current_task_id.rfind(kChainTaskPrefix, 0) != 0) {
        auto inserted = waiting_for_[blocker].insert(robot_id).second;
        ni->retry_after = this->now() + rclcpp::Duration::from_seconds(5.0);
        if (inserted) {
          PersistLogger::log_info("nav.wait_chain_active", robot_id, ni->current_task_id,
            "chain active, waiting for blocker " + blocker,
            __FILE__, __LINE__, __func__);
        }
        return;
      }

      // ── 检查阻塞者是否忙 ──
      auto blocker_ni = get_or_create_nav(blocker);
      bool blocker_busy = blocker_ni && (blocker_ni->has_active_goal || !blocker_ni->route.empty() || blocker_ni->chassis_task_sent);

      if (blocker_busy) {
        if (chain_plan_.active) {
          if (ni->current_task_id.rfind(kChainTaskPrefix, 0) == 0) {
            ni->retry_count++;
            ni->retry_after = this->now() + rclcpp::Duration::from_seconds(2.0);
            if (ni->retry_count > retry_max_) {
              abort_chain("chain step blocked by busy robot " + blocker);
            }
            return;
          }
          auto inserted = waiting_for_[blocker].insert(robot_id).second;
          ni->retry_after = this->now() + rclcpp::Duration::from_seconds(5.0);
          if (inserted) {
            PersistLogger::log_info("nav.wait_chain_active", robot_id, ni->current_task_id,
              "chain active, waiting for blocker " + blocker,
              __FILE__, __LINE__, __func__);
          }
          return;
        }

        // 互锁时即使阻塞者忙也尝试链撤退（暂停双方任务协调交换）
        if (is_mutual_block(blocker, wp, robot_id, from)) {
          chain_plan_.saved_task_ids.clear();
          chain_plan_.saved_targets.clear();
          if (!blocker_ni->current_task_id.empty()) {
            chain_plan_.saved_task_ids[blocker] = blocker_ni->current_task_id;
            if (!blocker_ni->route.empty())
              chain_plan_.saved_targets[blocker] = blocker_ni->route.back();
          }
          if (!ni->current_task_id.empty()) {
            chain_plan_.saved_task_ids[robot_id] = ni->current_task_id;
            if (!ni->route.empty())
              chain_plan_.saved_targets[robot_id] = ni->route.back();
          }
          // 只 block 未来 route(route_index 起,含当前 to_wp),过去 wp 是有效 retreat 候选
          size_t blk_start = std::min(ni->route_index, ni->route.size());
          std::set<std::string> blocked_set(ni->route.begin() + blk_start, ni->route.end());
          if (try_build_retreat_chain(robot_id, from, wp, blocker, blocked_set, 0)) {
            bool chain_valid = true;
            for (const auto & s : chain_plan_.steps) {
              auto test = traffic_->get_waypoint_pose(s.target_wp);
              if (test.position.x == 0.0 && test.position.y == 0.0 && test.position.z == 0.0) {
                auto all = traffic_->get_all_waypoint_poses();
                if (all.find(s.target_wp) == all.end()) { chain_valid = false; break; }
              }
            }
            if (chain_valid) {
              chain_plan_.original_requester = robot_id;
              chain_plan_.original_target = ni->route.back();
              chain_plan_.original_task_id = ni->current_task_id;
              chain_plan_.active = true;
              chain_plan_.started_at = this->now();
              chain_plan_.step_started_at = chain_plan_.started_at;
              chain_plan_.current_step = 0;
              chain_plan_.step_retry_count = 0;
              for (const auto & s : chain_plan_.steps) {
                auto step_ni = get_or_create_nav(s.robot_id);
                if (!step_ni || step_ni->current_task_id.empty() ||
                    chain_plan_.saved_task_ids.count(s.robot_id)) {
                  continue;
                }
                chain_plan_.saved_task_ids[s.robot_id] = step_ni->current_task_id;
                if (!step_ni->route.empty()) chain_plan_.saved_targets[s.robot_id] = step_ni->route.back();
              }
              cancel_goals(ni);
              cancel_goals(blocker_ni);
              ni->has_active_goal = false;
              ni->route.clear();
              ni->route_index = 0;
              ni->retry_count = 0;
              blocker_ni->has_active_goal = false;
              blocker_ni->route.clear();
              blocker_ni->route_index = 0;
              blocker_ni->retry_count = 0;
              PersistLogger::log_info("nav.chain_started_busy", robot_id, ni->current_task_id,
                "chain retreat for mutual block (busy blocker), " +
                std::to_string(chain_plan_.steps.size()) + " steps",
                __FILE__, __LINE__, __func__);
              execute_chain_step();
              return;
            }
            chain_plan_.steps.clear();
            chain_plan_.saved_task_ids.clear();
            chain_plan_.saved_targets.clear();
          }
        }
        // 非互锁 → 事件驱动等待：注册到阻塞者的等待列表
        auto inserted = waiting_for_[blocker].insert(robot_id).second;
        ni->retry_after = this->now() + rclcpp::Duration::from_seconds(30.0);
        if (inserted) {
          PersistLogger::log_info("nav.wait_for_blocker", robot_id, ni->current_task_id,
            "registered to wait for blocker " + blocker,
            __FILE__, __LINE__, __func__);
        }
        return;
      }

      // 阻塞者静止 → 正常退避协调
      ni->retry_count++;
      double jitter = 0.7 + 0.6 * (static_cast<double>(std::hash<std::string>{}(robot_id) % 1000) / 1000.0);
      double backoff = jitter * retry_base_ * std::pow(1.5, std::min(ni->retry_count, retry_max_));
      ni->retry_after = this->now() + rclcpp::Duration::from_seconds(backoff);
      auto waiting_inserted = waiting_for_[blocker].insert(robot_id).second;
      if (waiting_inserted) {
        PersistLogger::log_info("nav.wait_for_blocker", robot_id, ni->current_task_id,
          "registered to wait for blocker " + blocker,
          __FILE__, __LINE__, __func__);
      }

      if (is_internal_task_id(ni->current_task_id)) {
        bool blocker_truly_idle = blocker_ni &&
          blocker_ni->current_task_id.empty() && blocker_ni->route.empty();
        if (!chain_plan_.active && blocker_truly_idle) {
          std::set<std::string> final_exclude(ni->route.begin(), ni->route.end());
          for (const auto & existing : scheduler_->get_all_tasks()) {
            if (existing.task_id.empty() || existing.task_id == ni->current_task_id) continue;
            if (existing.waypoint_id.empty()) continue;
            if (existing.status == "completed" || existing.status == "failed" ||
                existing.status == "cancelled" || existing.status == "pending") {
              continue;
            }
            final_exclude.insert(existing.waypoint_id);
          }
          std::vector<std::string> avoid_path;
          double best_cost = std::numeric_limits<double>::max();
          auto all_wps = traffic_->get_all_waypoint_poses();
          auto from_pose = traffic_->get_waypoint_pose(wp);
          for (const auto & [candidate, pose] : all_wps) {
            if (candidate == wp || final_exclude.count(candidate)) continue;
            if (!occupancy_->is_zone_free_for(blocker, candidate)) continue;
            auto path = traffic_->find_path(wp, candidate);
            if (path.size() < 2) continue;
            bool blocked_path = false;
            for (size_t i = 1; i < path.size(); ++i) {
              auto hop_blocker = occupancy_->can_enter(blocker, path[i - 1], path[i]);
              if (!hop_blocker.empty() && hop_blocker != blocker) {
                blocked_path = true;
                break;
              }
            }
            if (blocked_path) continue;
            const double dist = std::hypot(
              pose.position.x - from_pose.position.x,
              pose.position.y - from_pose.position.y);
            const double cost = static_cast<double>(path.size()) * 10.0 + dist;
            if (cost < best_cost) {
              best_cost = cost;
              avoid_path = path;
            }
          }
          if (avoid_path.size() >= 2) {
            PersistLogger::log_info("nav.internal_avoid_push_blocker", robot_id, ni->current_task_id,
              "asking idle blocker " + blocker + " to move path=" + join_waypoints(avoid_path),
              __FILE__, __LINE__, __func__);
            blocker_ni->route = avoid_path;
            blocker_ni->route_index = 0;
            blocker_ni->route_alignment_done = false;
            blocker_ni->current_task_id = "avoidance_" + blocker;
            blocker_ni->retry_count = 0;
            blocker_ni->retry_after = rclcpp::Time{};
            navigate_to_next_waypoint(blocker);
            return;
          }
        }
        if (chain_plan_.active && ni->retry_count > retry_max_) {
          abort_chain("internal coordination step blocked from " + from + " to " + wp +
            " by " + blocker);
        } else if (!chain_plan_.active && ni->retry_count > retry_max_) {
          occupancy_->release_reservations(robot_id);
          ni->current_task_id.clear();
          ni->route.clear();
          ni->route_index = 0;
          ni->retry_count = 0;
        }
        return;
      }

      if (ni->retry_count >= 2 && !is_internal_task_id(ni->current_task_id)) {
        auto blocker_st = robots_.find(blocker);
        bool blocker_online = blocker_st != robots_.end() &&
          blocker_st->second.connection_status == "online";

        if (!blocker_online) {
          PersistLogger::log_warn("nav.blocker_offline", robot_id, ni->current_task_id,
            "blocker " + blocker + " is offline",
            __FILE__, __LINE__, __func__);
        } else {
          bool mutual = is_mutual_block(blocker, wp, robot_id, from);
          if (mutual) {
            // 保存参与底盘的原始任务 (允许多链并发)
            chain_plan_.saved_task_ids.clear();
            chain_plan_.saved_targets.clear();
            if (!blocker_ni->current_task_id.empty()) {
              chain_plan_.saved_task_ids[blocker] = blocker_ni->current_task_id;
              if (!blocker_ni->route.empty())
                chain_plan_.saved_targets[blocker] = blocker_ni->route.back();
            }
            if (!ni->current_task_id.empty()) {
              chain_plan_.saved_task_ids[robot_id] = ni->current_task_id;
              if (!ni->route.empty())
                chain_plan_.saved_targets[robot_id] = ni->route.back();
            }

            // 只 block 未来 route,过去 wp 可作为 retreat
            size_t blk_start = std::min(ni->route_index, ni->route.size());
            std::set<std::string> blocked_set(ni->route.begin() + blk_start, ni->route.end());
            if (try_build_retreat_chain(robot_id, from, wp, blocker, blocked_set, 0)) {
              // 链预验证: 所有目标航点必须存在
              bool chain_valid = true;
              for (const auto & s : chain_plan_.steps) {
                auto test = traffic_->get_waypoint_pose(s.target_wp);
                if (test.position.x == 0.0 && test.position.y == 0.0 && test.position.z == 0.0) {
                  auto all = traffic_->get_all_waypoint_poses();
                  if (all.find(s.target_wp) == all.end()) {
                    chain_valid = false; break;
                  }
                }
              }
              if (chain_valid) {
                chain_plan_.original_requester = robot_id;
                chain_plan_.original_target = ni->route.back();
                chain_plan_.original_task_id = ni->current_task_id;
                chain_plan_.active = true;
                chain_plan_.started_at = this->now();
                chain_plan_.step_started_at = chain_plan_.started_at;
                chain_plan_.current_step = 0;
                chain_plan_.step_retry_count = 0;
                for (const auto & s : chain_plan_.steps) {
                  auto step_ni = get_or_create_nav(s.robot_id);
                  if (!step_ni || step_ni->current_task_id.empty() ||
                      chain_plan_.saved_task_ids.count(s.robot_id)) {
                    continue;
                  }
                  chain_plan_.saved_task_ids[s.robot_id] = step_ni->current_task_id;
                  if (!step_ni->route.empty()) chain_plan_.saved_targets[s.robot_id] = step_ni->route.back();
                }

                // 暂停请求者和阻塞者的导航
                cancel_goals(ni);
                cancel_goals(blocker_ni);
                ni->has_active_goal = false;
                ni->route.clear();
                ni->route_index = 0;
                ni->retry_count = 0;
                blocker_ni->has_active_goal = false;
                blocker_ni->route.clear();
                blocker_ni->route_index = 0;
                blocker_ni->retry_count = 0;

                PersistLogger::log_info("nav.chain_started", robot_id, ni->current_task_id,
                  "chain retreat initiated, " + std::to_string(chain_plan_.steps.size()) + " steps",
                  __FILE__, __LINE__, __func__);
                execute_chain_step();
                return;
              }
              chain_plan_.steps.clear();
              chain_plan_.saved_task_ids.clear();
              chain_plan_.saved_targets.clear();
            } else {
              // try_build_retreat_chain 或验证失败
              chain_plan_.steps.clear();
              chain_plan_.saved_task_ids.clear();
              chain_plan_.saved_targets.clear();
              PersistLogger::log_warn("nav.chain_build_failed", robot_id, ni->current_task_id,
                "could not build retreat chain from " + from + " to " + wp +
                " blocked by " + blocker,
                __FILE__, __LINE__, __func__);
            }
          }

          // 简单避让(适用于空闲阻塞者; 链成功时已在上面 return，不会到这里)
          bool blocker_truly_idle = blocker_ni &&
            blocker_ni->current_task_id.empty() && blocker_ni->route.empty();
          if (blocker_truly_idle) {
            // 统一图感知安全搜索
            std::set<std::string> exclude_set(ni->route.begin(), ni->route.end());
            for (const auto & existing : scheduler_->get_all_tasks()) {
              if (existing.task_id.empty() || existing.task_id == ni->current_task_id) continue;
              if (existing.waypoint_id.empty()) continue;
              if (existing.status == "completed" || existing.status == "failed" ||
                  existing.status == "cancelled" || existing.status == "pending") {
                continue;
              }
              exclude_set.insert(existing.waypoint_id);
            }
            std::string avoid_wp = find_safe_free_waypoint(wp, exclude_set, blocker);
            auto avoid_path = avoid_wp.empty() ? std::vector<std::string>{} :
              traffic_->find_path(wp, avoid_wp);
            if (avoid_path.size() >= 2) {
              // 先预留目标航点，防止其他机器人抢占
              if (!occupancy_->reserve_next(blocker, avoid_path[0], avoid_path[1])) {
                PersistLogger::log_warn("nav.avoid_reserve_failed", robot_id, ni->current_task_id,
                  "could not reserve " + avoid_wp + " for blocker " + blocker,
                  __FILE__, __LINE__, __func__);
              } else {
                PersistLogger::log_info("nav.avoid_idle_blocker", robot_id, ni->current_task_id,
                  "asking idle blocker " + blocker + " to move from " + wp + " to " + avoid_wp,
                  __FILE__, __LINE__, __func__);
                blocker_ni->route = avoid_path;
                blocker_ni->route_index = 0;
                blocker_ni->route_alignment_done = false;
                blocker_ni->current_task_id = "avoidance_" + blocker;
                blocker_ni->retry_count = 0;
                blocker_ni->retry_after = rclcpp::Time{};
                navigate_to_next_waypoint(blocker);
                return;
              }
            }
            if (!chain_plan_.active) {
              chain_plan_.saved_task_ids.clear();
              chain_plan_.saved_targets.clear();
              if (!ni->current_task_id.empty()) {
                chain_plan_.saved_task_ids[robot_id] = ni->current_task_id;
                if (!ni->route.empty())
                  chain_plan_.saved_targets[robot_id] = ni->route.back();
              }
              // 只 block 未来 route,过去 wp 可作为 retreat
              size_t blk_start = std::min(ni->route_index, ni->route.size());
              std::set<std::string> blocked_set(ni->route.begin() + blk_start, ni->route.end());
              for (const auto & existing : scheduler_->get_all_tasks()) {
                if (existing.task_id.empty() || existing.task_id == ni->current_task_id) continue;
                if (existing.waypoint_id.empty()) continue;
                if (existing.status == "completed" || existing.status == "failed" ||
                    existing.status == "cancelled" || existing.status == "pending") {
                  continue;
                }
                blocked_set.insert(existing.waypoint_id);
              }
              if (try_build_retreat_chain(robot_id, from, wp, blocker, blocked_set, 0)) {
                bool chain_valid = true;
                for (const auto & s : chain_plan_.steps) {
                  auto test = traffic_->get_waypoint_pose(s.target_wp);
                  if (test.position.x == 0.0 && test.position.y == 0.0 && test.position.z == 0.0) {
                    auto all = traffic_->get_all_waypoint_poses();
                    if (all.find(s.target_wp) == all.end()) { chain_valid = false; break; }
                  }
                }
                if (chain_valid) {
                  chain_plan_.original_requester = robot_id;
                  chain_plan_.original_target = ni->route.empty() ? std::string{} : ni->route.back();
                  chain_plan_.original_task_id = ni->current_task_id;
                  chain_plan_.active = true;
                  chain_plan_.started_at = this->now();
                  chain_plan_.step_started_at = chain_plan_.started_at;
                  chain_plan_.current_step = 0;
                  chain_plan_.step_retry_count = 0;
                  for (const auto & s : chain_plan_.steps) {
                    auto step_ni = get_or_create_nav(s.robot_id);
                    if (!step_ni || step_ni->current_task_id.empty() ||
                        chain_plan_.saved_task_ids.count(s.robot_id)) {
                      continue;
                    }
                    chain_plan_.saved_task_ids[s.robot_id] = step_ni->current_task_id;
                    if (!step_ni->route.empty()) chain_plan_.saved_targets[s.robot_id] = step_ni->route.back();
                  }
                  cancel_goals(ni);
                  cancel_goals(blocker_ni);
                  ni->has_active_goal = false;
                  ni->route.clear();
                  ni->route_index = 0;
                  ni->retry_count = 0;
                  blocker_ni->has_active_goal = false;
                  blocker_ni->route.clear();
                  blocker_ni->route_index = 0;
                  blocker_ni->retry_count = 0;
                  PersistLogger::log_info("nav.chain_started_idle_blocker", robot_id, ni->current_task_id,
                    "chain clear for idle blocker " + blocker + ", " +
                    std::to_string(chain_plan_.steps.size()) + " steps",
                    __FILE__, __LINE__, __func__);
                  execute_chain_step();
                  return;
                }
                chain_plan_.steps.clear();
                chain_plan_.saved_task_ids.clear();
                chain_plan_.saved_targets.clear();
              }
            }
          }      // end if (blocker_truly_idle)
        } // end else (blocker_online)
      } // end if retry_count >= 2

      if (ni->retry_count >= 3 && !is_internal_task_id(ni->current_task_id)) {
        std::set<std::string> retreat_exclude(ni->route.begin(), ni->route.end());
        retreat_exclude.insert(wp);
        for (const auto & existing : scheduler_->get_all_tasks()) {
          if (existing.task_id.empty() || existing.task_id == ni->current_task_id) continue;
          if (existing.waypoint_id.empty()) continue;
          if (existing.status == "completed" || existing.status == "failed" ||
              existing.status == "cancelled" || existing.status == "pending") {
            continue;
          }
          retreat_exclude.insert(existing.waypoint_id);
        }
        std::string retreat_wp = find_safe_free_waypoint(from, retreat_exclude, robot_id);
        auto retreat_path = retreat_wp.empty() ? std::vector<std::string>{} :
          traffic_->find_path(from, retreat_wp);
        if (retreat_path.size() >= 2) {
          occupancy_->release_reservations(robot_id);
          if (occupancy_->reserve_next(robot_id, retreat_path[0], retreat_path[1])) {
            std::string tid = ni->current_task_id;
            scheduler_->mark_task_waiting(tid);
            clear_task_wait_condition(tid);
            task_waits_[tid] = {TaskWaitState::SELF_RELOCATING, robot_id, tid,
              blocker, from, wp, ni->route.empty() ? std::string{} : ni->route.back(),
              ni->retry_count};
            waiting_for_[blocker].insert(robot_id);
            fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
            if (!ti.task_id.empty()) task_pub_->publish(ti);
            PersistLogger::log_info("nav.self_retreat", robot_id, tid,
              "retreating from " + from + " to " + retreat_wp +
              " after repeated block by " + blocker,
              __FILE__, __LINE__, __func__);
            ni->has_active_goal = false;
            ni->route = retreat_path;
            ni->route_index = 0;
            ni->route_alignment_done = false;
            ni->current_task_id = "avoidance_" + robot_id;
            ni->retry_count = 0;
            ni->retry_after = rclcpp::Time{};
            navigate_to_next_waypoint(robot_id);
            return;
          }
        }
      }

      // 退避耗尽 → 释放非固定绑定回队
      if (ni->retry_count > retry_max_ * 2) {
        std::string tid = ni->current_task_id;
        occupancy_->release_reservations(robot_id);
        ni->current_task_id.clear();
        ni->route.clear();
        ni->route_index = 0;
        ni->retry_count = 0;
        if (scheduler_->would_exceed_retry_cycles(tid, max_task_retry_cycles_)) {
          scheduler_->fail_task(tid, "hop blocked, max retry cycles exceeded");
          fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
          task_pub_->publish(ti);
          finalize_task_completion(robot_id, tid);
        } else {
          scheduler_->mark_task_pending_retry(tid);
          fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
          task_pub_->publish(ti);
        }
      }
      return;
    }

    if (!occupancy_->reserve_next(robot_id, from, wp)) {
      ni->retry_count++;
      ni->retry_after = this->now() + rclcpp::Duration::from_seconds(retry_base_);
      return;
    }
    // 双跳预留：再预留下一跳（跳+1），给后方机器人留缓冲区间
    if (target + 1 < n) {
      std::string next_wp = ni->route[target + 1];
      std::string lookahead_blocker = occupancy_->can_enter(robot_id, wp, next_wp);
      if (!lookahead_blocker.empty() && lookahead_blocker != robot_id) {
        bool lookahead_blocker_busy = true;
        if (lookahead_blocker != "invalid") {
          auto lookahead_blocker_ni = get_or_create_nav(lookahead_blocker);
          lookahead_blocker_busy = lookahead_blocker_ni &&
            (lookahead_blocker_ni->has_active_goal ||
             !lookahead_blocker_ni->current_task_id.empty() ||
             !lookahead_blocker_ni->route.empty() ||
             lookahead_blocker_ni->chassis_task_sent);
        }
        std::string lookahead_blocker_wp;
        if (!lookahead_blocker_busy && lookahead_blocker != "invalid") {
          auto blocker_st = robots_.find(lookahead_blocker);
          if (blocker_st != robots_.end()) {
            lookahead_blocker_wp = blocker_st->second.current_waypoint;
            if (lookahead_blocker_wp.empty()) {
              lookahead_blocker_wp = traffic_->find_nearest_waypoint(blocker_st->second.current_pose);
            }
          }
        }
        if (!lookahead_blocker_busy && lookahead_blocker != "invalid" &&
            lookahead_blocker_wp == wp) {
          std::set<std::string> yield_exclude;
          for (size_t i = target; i < ni->route.size(); ++i) {
            yield_exclude.insert(ni->route[i]);
          }
          for (const auto & existing : scheduler_->get_all_tasks()) {
            if (existing.task_id.empty() || existing.task_id == ni->current_task_id) continue;
            if (existing.waypoint_id.empty()) continue;
            if (existing.status == "completed" || existing.status == "failed" ||
                existing.status == "cancelled" || existing.status == "pending") {
              continue;
            }
            yield_exclude.insert(existing.waypoint_id);
          }
          std::string yield_wp = find_safe_free_waypoint(from, yield_exclude, robot_id);
          auto yield_path = yield_wp.empty() ? std::vector<std::string>{} :
            traffic_->find_path(from, yield_wp);
          if (yield_path.size() >= 2) {
            occupancy_->release_reservations(robot_id);
            if (occupancy_->reserve_next(robot_id, yield_path[0], yield_path[1])) {
              std::string tid = ni->current_task_id;
              scheduler_->mark_task_waiting(tid);
              clear_task_wait_condition(tid);
              task_waits_[tid] = {TaskWaitState::SELF_RELOCATING, robot_id, tid,
                lookahead_blocker, from, wp, ni->route.empty() ? std::string{} : ni->route.back(),
                ni->retry_count};
              waiting_for_[lookahead_blocker].insert(robot_id);
              fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
              if (!ti.task_id.empty()) task_pub_->publish(ti);
              PersistLogger::log_info("nav.lookahead_idle_blocker_yield", robot_id, tid,
                "yielding from " + from + " to " + yield_wp + " before " + wp +
                " due to idle lookahead blocker " + lookahead_blocker,
                __FILE__, __LINE__, __func__);
              ni->has_active_goal = false;
              ni->route = yield_path;
              ni->route_index = 0;
              ni->route_alignment_done = false;
              ni->current_task_id = "avoidance_" + robot_id;
              ni->retry_count = 0;
              ni->retry_after = rclcpp::Time{};
              navigate_to_next_waypoint(robot_id);
              return;
            }
          }
          PersistLogger::log_info("nav.lookahead_idle_blocker_soft", robot_id, ni->current_task_id,
            "from=" + wp + " to=" + next_wp + " idle_blocker=" + lookahead_blocker,
            __FILE__, __LINE__, __func__);
        } else if (!lookahead_blocker_busy && lookahead_blocker != "invalid") {
          PersistLogger::log_info("nav.lookahead_idle_blocker_soft", robot_id, ni->current_task_id,
            "from=" + wp + " to=" + next_wp + " idle_blocker=" + lookahead_blocker,
            __FILE__, __LINE__, __func__);
        } else {
          if (lookahead_blocker != "invalid") {
            waiting_for_[lookahead_blocker].insert(robot_id);
          }
          PersistLogger::log_info("nav.lookahead_blocked_soft", robot_id, ni->current_task_id,
            "from=" + wp + " to=" + next_wp + " blocker=" + lookahead_blocker,
            __FILE__, __LINE__, __func__);
        }
      }
      if (lookahead_blocker.empty() && !occupancy_->reserve_next(robot_id, wp, next_wp)) {
        occupancy_->release_reservations(robot_id);
        ni->retry_count++;
        ni->retry_after = this->now() + rclcpp::Duration::from_seconds(retry_base_);
        return;
      }
    }
    ni->retry_count = 0;
  }

  ni->route_index = target;
  navigate_to_waypoint(robot_id, wp, ni->current_task_id, is_final);
}

// ============================================================================
// 发送 NavigateToPose action goal
// ============================================================================

void FleetManagerNode::navigate_to_waypoint(
  const std::string & robot_id,
  const std::string & wp_id,
  const std::string & task_id,
  bool is_final,
  bool alignment_done)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);
  if (!ni) return;

  const auto now = this->now();
  if (ni->recent_cancel_until.nanoseconds() > 0 && now < ni->recent_cancel_until) {
    ni->retry_after = ni->recent_cancel_until;
    return;
  }

  // 取消旧 goal
  if (ni->goal_handle && ni->nav_client) {
    ni->nav_client->async_cancel_goal(ni->goal_handle);
    ni->goal_handle.reset();
    ni->has_active_goal = false;
    ni->nav_seq++;
  }

  auto wp_pose = traffic_->get_waypoint_pose(wp_id);
  auto st = robots_.find(robot_id);

  if (!alignment_done && !ni->route_alignment_done) {
    if (st == robots_.end() || st->second.connection_status != "online") {
      ni->retry_after = now + rclcpp::Duration::from_seconds(0.5);
      return;
    }

    const double dx = wp_pose.position.x - st->second.current_pose.position.x;
    const double dy = wp_pose.position.y - st->second.current_pose.position.y;
    const double dist = std::hypot(dx, dy);
    if (dist > 1e-6) {
      if (!ni->align_client || !ni->align_client->action_server_is_ready()) {
        PersistLogger::log_warn("nav.align_not_ready", robot_id, task_id,
          "NavigateThroughPoses server not ready", __FILE__, __LINE__, __func__);
        ni->retry_after = now + rclcpp::Duration::from_seconds(0.5);
        return;
      }

      geometry_msgs::msg::PoseStamped align_pose;
      align_pose.header.frame_id = "map";
      align_pose.header.stamp = now;
      align_pose.pose.position = st->second.current_pose.position;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, std::atan2(dy, dx));
      q.normalize();
      align_pose.pose.orientation.x = q.x();
      align_pose.pose.orientation.y = q.y();
      align_pose.pose.orientation.z = q.z();
      align_pose.pose.orientation.w = q.w();

      auto align_goal = NavigateThroughPoses::Goal();
      align_goal.poses.push_back(align_pose);
      align_goal.poses.push_back(align_pose);

      auto align_opts = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
      uint64_t seq = ni->nav_seq;

      align_opts.goal_response_callback =
        [this, robot_id, task_id, seq](GoalHandleNavigateThrough::SharedPtr gh) {
          std::lock_guard<std::recursive_mutex> l(mtx_);
          auto n = get_or_create_nav(robot_id);
          if (!n || n->nav_seq != seq || n->current_task_id != task_id) return;
          if (!gh) {
            n->has_active_goal = false;
            n->aligning_before_nav = false;
            PersistLogger::log_warn("nav.align_rejected", robot_id, n->current_task_id,
              "NavigateThroughPoses rejected", __FILE__, __LINE__, __func__);
          } else {
            n->align_goal_handle = gh;
            n->nav_last_activity = this->now();
          }
        };

      align_opts.result_callback =
        [this, robot_id, wp_id, task_id, is_final, seq](
          const GoalHandleNavigateThrough::WrappedResult & r) {
          std::lock_guard<std::recursive_mutex> l(mtx_);
          auto n = get_or_create_nav(robot_id);
          if (!n || n->nav_seq != seq || n->current_task_id != task_id) return;

          n->has_active_goal = false;
          n->aligning_before_nav = false;
          n->align_goal_handle.reset();

          if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
            n->route_alignment_done = true;
            PersistLogger::log_info("nav.align_succeeded", robot_id, task_id,
              "aligned before navigating to " + wp_id, __FILE__, __LINE__, __func__);
            navigate_to_waypoint(robot_id, wp_id, task_id, is_final, true);
          } else if (r.code == rclcpp_action::ResultCode::CANCELED) {
          } else {
            PersistLogger::log_warn("nav.align_failed", robot_id, task_id,
              "align result code=" + std::to_string(static_cast<int>(r.code)),
              __FILE__, __LINE__, __func__);
            n->retry_count++;
            if (n->retry_count <= 3) {
              navigate_to_next_waypoint(robot_id);
            } else {
              std::string tid = task_id;
              occupancy_->release_reservations(robot_id);
              n->current_task_id.clear();
              n->route.clear();
              n->route_index = 0;
              n->retry_count = 0;
              if (scheduler_->would_exceed_retry_cycles(tid, max_task_retry_cycles_)) {
                scheduler_->fail_task(tid, "alignment failed repeatedly, max retry cycles exceeded");
                fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
                if (!ti.task_id.empty()) task_pub_->publish(ti);
                finalize_task_completion(robot_id, tid);
              } else {
                scheduler_->mark_task_pending_retry(tid);
                fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
                if (!ti.task_id.empty()) task_pub_->publish(ti);
              }
            }
          }
        };

      ni->has_active_goal = true;
      ni->aligning_before_nav = true;
      ni->nav_since = now;
      ni->nav_last_activity = now;
      PersistLogger::log_info("nav.align_start", robot_id, task_id,
        "aligning toward " + wp_id, __FILE__, __LINE__, __func__);
      ni->align_client->async_send_goal(align_goal, align_opts);
      return;
    }
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = now;
  goal.pose.position = wp_pose.position;
  if (st != robots_.end()) {
    const auto & q = st->second.current_pose.orientation;
    double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm > 1e-6) {
      goal.pose.orientation.x = q.x / norm;
      goal.pose.orientation.y = q.y / norm;
      goal.pose.orientation.z = q.z / norm;
      goal.pose.orientation.w = q.w / norm;
    } else {
      goal.pose.orientation.w = 1.0;
    }
  } else {
    goal.pose.orientation.w = 1.0;
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = goal;

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  uint64_t seq = ni->nav_seq;

  opts.goal_response_callback =
    [this, robot_id, task_id, seq](GoalHandleNavigate::SharedPtr gh) {
      std::lock_guard<std::recursive_mutex> l(mtx_);
      auto n = get_or_create_nav(robot_id);
      if (!n || n->nav_seq != seq || n->current_task_id != task_id) return;
      if (!gh) {
        n->has_active_goal = false;
        PersistLogger::log_warn("nav.rejected", robot_id, n->current_task_id,
          "NavigateToPose rejected", __FILE__, __LINE__, __func__);
      } else {
        n->goal_handle = gh;
        n->nav_last_activity = this->now();
      }
    };

  opts.result_callback =
    [this, robot_id, task_id, is_final, seq](const GoalHandleNavigate::WrappedResult & r) {
      std::lock_guard<std::recursive_mutex> l(mtx_);
      auto n = get_or_create_nav(robot_id);
      if (!n || n->nav_seq != seq || n->current_task_id != task_id) return;

      n->has_active_goal = false;
      n->goal_handle.reset();

      if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
        n->retry_count = 0;
        // 链步骤完成 → 走链回调(仅最终航点 reached 时触发)
        if (is_final && chain_plan_.active && task_id.rfind(kChainTaskPrefix, 0) == 0) {
          on_chain_step_complete(robot_id, true);
          return;
        }
        if (is_final) {
          on_nav_succeeded(robot_id, task_id);
        } else {
          navigate_to_next_waypoint(robot_id);
        }
      } else if (r.code == rclcpp_action::ResultCode::CANCELED) {
        // 主动取消, 不做额外处理
      } else {
        // 失败: 最多重试 3 次, 超限则保留绑定回队
        PersistLogger::log_warn("nav.failed", robot_id, task_id,
          "nav result code=" + std::to_string(static_cast<int>(r.code)),
          __FILE__, __LINE__, __func__);
        n->retry_count++;
        if (n->retry_count <= 3) {
          navigate_to_next_waypoint(robot_id);
        } else {
          std::string tid = task_id;
          occupancy_->release_reservations(robot_id);
          n->current_task_id.clear();
          n->route.clear();
          n->route_index = 0;
          n->retry_count = 0;
          if (scheduler_->would_exceed_retry_cycles(tid, max_task_retry_cycles_)) {
            scheduler_->fail_task(tid, "nav failed repeatedly, max retry cycles exceeded");
            fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
            if (!ti.task_id.empty()) task_pub_->publish(ti);
            finalize_task_completion(robot_id, tid);
          } else {
            scheduler_->mark_task_pending_retry(tid);
            fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
            if (!ti.task_id.empty()) task_pub_->publish(ti);
          }
        }
      }
    };

  if (!ni->nav_client || !ni->nav_client->action_server_is_ready()) {
    PersistLogger::log_warn("nav.not_ready", robot_id, task_id,
      "NavigateToPose server not ready", __FILE__, __LINE__, __func__);
    ni->retry_after = now + rclcpp::Duration::from_seconds(0.5);
    return;
  }

  ni->has_active_goal = true;
  ni->aligning_before_nav = false;
  ni->nav_since = now;
  ni->nav_last_activity = now;
  ni->recent_cancel_until = now + rclcpp::Duration::from_seconds(kNavCancelSettlingSec);

  ni->nav_client->async_send_goal(goal_msg, opts);
}

// ============================================================================
// 导航成功回调
// ============================================================================

void FleetManagerNode::on_nav_succeeded(
  const std::string & robot_id, const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);
  clear_task_wait_condition(task_id);

  // 内部任务(避让/链撤退/自动驶离): 仅清理 nav 状态, 不触发完整任务流程
  if (task_id.rfind("avoidance_", 0) == 0 ||
      task_id.rfind(kChainTaskPrefix, 0) == 0 ||
      task_id.rfind("relocate_", 0) == 0) {
    if (ni) {
      occupancy_->release_reservations(robot_id);
      ni->current_task_id.clear();
      ni->has_active_goal = false;
      ni->route.clear();
      ni->route_index = 0;
    }
    wake_waiters(robot_id);
    return;
  }

  // 用户任务: type=0/1 为 CRUISE(直接完成), 其余转底盘执行
  uint8_t type = scheduler_->get_task_type(task_id);

  if (type == 1 || type == 0) {
    scheduler_->complete_task(task_id);
    fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
    task_pub_->publish(ti);
    finalize_task_completion(robot_id, task_id);
    wake_waiters(robot_id);  // 唤醒等待此机器人的所有请求者
  } else {
    scheduler_->mark_task_executing(task_id);
    fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
    task_pub_->publish(ti);

    auto info = scheduler_->get_task_info(task_id);
    uint16_t wp_num = wp_to_u16(info.waypoint_id);
    send_chassis_cmd(robot_id, task_id, wp_num, type, info.site_code);
  }
}

// ============================================================================
// 到达检测 — 卡住超时 20s 重试, 绝对超时 45s 回队列
// ============================================================================

void FleetManagerNode::check_arrivals()
{
  // 链整体超时检测
  if (chain_plan_.active) {
    const auto now = this->now();
    if ((now - chain_plan_.started_at).seconds() >= kChainTotalTimeout) {
      abort_chain("chain total timeout (" + std::to_string(kChainTotalTimeout) + "s)");
      return;
    }
    if (chain_plan_.step_started_at.nanoseconds() == 0) {
      chain_plan_.step_started_at = now;
    }
    if (chain_plan_.current_step < chain_plan_.steps.size() &&
        (now - chain_plan_.step_started_at).seconds() >= kChainStepTimeout) {
      const auto & step = chain_plan_.steps[chain_plan_.current_step];
      auto ni = get_or_create_nav(step.robot_id);
      if (!ni || !ni->has_active_goal) {
        abort_chain("chain step made no progress for " + std::to_string(kChainStepTimeout) + "s");
        return;
      }
    }
  }

  const auto now = this->now();
  for (auto & [rid, ni] : navs_) {
    if (!ni || !ni->has_active_goal) continue;
    if (ni->route.empty()) continue;

    // === 位置到达检测: 物理位置在目标航点半径内 → 直接完成(不等 Nav2 orientation) ===
    auto st = robots_.find(rid);
    if (!ni->aligning_before_nav && st != robots_.end()) {
      size_t target_index = std::min(ni->route_index, ni->route.size() - 1);
      std::string target_wp = ni->route[target_index];
      auto target_pose = traffic_->get_waypoint_pose(target_wp);
      double dx = st->second.current_pose.position.x - target_pose.position.x;
      double dy = st->second.current_pose.position.y - target_pose.position.y;
      if (std::hypot(dx, dy) <= waypoint_radius_) {
        std::string task_id = ni->current_task_id;
        bool is_final = (target_index + 1 >= ni->route.size());
        PersistLogger::log_info("nav.position_arrived", rid, ni->current_task_id,
          "physically at " + target_wp + ", completing navigation",
          __FILE__, __LINE__, __func__);
        cancel_goals(ni);
        stop_robot(rid, 20);  // 立刻停住底盘，防止 Nav2 取消回调前的方向震荡
        ni->has_active_goal = false;
        if (is_final && chain_plan_.active && task_id.rfind(kChainTaskPrefix, 0) == 0) {
          on_chain_step_complete(rid, true);
        } else if (is_final) {
          on_nav_succeeded(rid, task_id);
        } else {
          ni->route_index = target_index + 1;
          ni->retry_after = ni->recent_cancel_until;
          if (chain_plan_.active && task_id.rfind(kChainTaskPrefix, 0) == 0) {
            chain_plan_.step_started_at = now;
          }
          navigate_to_next_waypoint(rid);
        }
        continue;
      }
    }

    // 卡住超时: 取消并重试(最多 3 次), 超限回队
    if (ni->nav_since.nanoseconds() > 0 && nav_stuck_timeout_ > 0 &&
        (now - ni->nav_since).seconds() > nav_stuck_timeout_) {
      cancel_goals(ni);
      ni->has_active_goal = false;
      ni->retry_count++;
      if (ni->retry_count <= 3) {
        navigate_to_next_waypoint(rid);
      } else {
        std::string tid = ni->current_task_id;
        occupancy_->release_reservations(rid);
        ni->current_task_id.clear();
        ni->route.clear();
        ni->route_index = 0;
        ni->retry_count = 0;
        if (scheduler_->would_exceed_retry_cycles(tid, max_task_retry_cycles_)) {
          scheduler_->fail_task(tid, "stuck navigation, max retry cycles exceeded");
          fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
          task_pub_->publish(ti);
          finalize_task_completion(rid, tid);
        } else {
          scheduler_->mark_task_pending_retry(tid);
          fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
          task_pub_->publish(ti);
        }
      }
    }

    // 绝对超时: 无条件回队
    if (ni->nav_since.nanoseconds() > 0 && nav_absolute_timeout_ > 0 &&
        (now - ni->nav_since).seconds() > nav_absolute_timeout_) {
      cancel_goals(ni);
      ni->has_active_goal = false;
      std::string tid = ni->current_task_id;
      occupancy_->release_reservations(rid);
      ni->current_task_id.clear();
      ni->route.clear();
      ni->route_index = 0;
      if (scheduler_->would_exceed_retry_cycles(tid, max_task_retry_cycles_)) {
        scheduler_->fail_task(tid, "nav absolute timeout, max retry cycles exceeded");
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
        task_pub_->publish(ti);
        finalize_task_completion(rid, tid);
      } else {
        scheduler_->mark_task_pending_retry(tid);
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
        task_pub_->publish(ti);
      }
    }
  }
}

// ============================================================================
// 底盘任务控制
// ============================================================================

void FleetManagerNode::send_chassis_cmd(
  const std::string & robot_id, const std::string & task_id,
  uint16_t wp_num, uint8_t type, uint32_t site)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);
  if (!ni || !ni->task_cmd_pub) return;

  uint64_t task_num = std::hash<std::string>{}(task_id);
  fleet_msgs::msg::TaskCmd cmd;
  cmd.task_id     = task_num;
  cmd.waypoint_id = wp_num;
  cmd.task_type   = type;
  cmd.site_code   = site;
  cmd.ack         = false;

  ni->task_cmd_pub->publish(cmd);
  ni->chassis_task_sent  = true;
  ni->chassis_handshake_ok = false;
  ni->chassis_acked      = false;
  ni->chassis_hs_deadline = this->now() + rclcpp::Duration::from_seconds(chassis_hs_timeout_);
  ni->chassis_exec_deadline = this->now() + rclcpp::Duration::from_seconds(chassis_hs_timeout_ + chassis_exec_timeout_);
  ni->pending_task_type = type;
  ni->pending_site_code = site;
  ni->pending_wp_num    = wp_num;
  ni->pending_task_num  = task_num;
}

void FleetManagerNode::send_chassis_ack(const std::string & robot_id, uint64_t task_num)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);
  if (!ni || !ni->task_cmd_pub) return;

  fleet_msgs::msg::TaskCmd ack;
  ack.task_id = task_num;
  ack.ack = true;
  ni->task_cmd_pub->publish(ack);
}

void FleetManagerNode::chassis_fb_callback(
  const std::string & robot_id, const fleet_msgs::msg::TaskFb::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);
  if (!ni) return;

  const uint64_t fb_num = msg->task_id;
  const uint8_t status  = msg->status;

  // 已完成任务的残余反馈: 仅回复 ack
  if (!ni->chassis_task_sent) {
    if (ni->chassis_acked && (status == 1 || status == 2))
      send_chassis_ack(robot_id, fb_num);
    return;
  }

  // 握手已完成的重复反馈: 回复 ack
  if (status == 0 && ni->chassis_handshake_ok) {
    send_chassis_ack(robot_id, fb_num);
    return;
  }

  // 任务 ID 不匹配(旧命令): 回复 ack 并忽略
  if (fb_num != ni->pending_task_num) {
    send_chassis_ack(robot_id, fb_num);
    return;
  }

  std::string task_id = ni->current_task_id;

  switch (status) {
    case 0: { // 握手成功
      ni->chassis_handshake_ok = true;
      ni->chassis_retries = 0;
      send_chassis_ack(robot_id, fb_num);
      break;
    }
    case 1: { // 执行完成
      send_chassis_ack(robot_id, fb_num);
      ni->chassis_task_sent = false;
      ni->chassis_handshake_ok = false;
      ni->chassis_retries = 0;
      ni->chassis_acked = true;
      scheduler_->complete_task(task_id);
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
      task_pub_->publish(ti);
      finalize_task_completion(robot_id, task_id);
      wake_waiters(robot_id);
      break;
    }
    case 2: { // 执行错误
      send_chassis_ack(robot_id, fb_num);
      ni->chassis_task_sent = false;
      ni->chassis_handshake_ok = false;
      ni->chassis_acked = true;
      scheduler_->fail_task(task_id, "chassis error 0x" + std::to_string(msg->error_code));
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
      task_pub_->publish(ti);
      finalize_task_completion(robot_id, task_id);
      break;
    }
    default: break;
  }
}

void FleetManagerNode::chassis_timeout_check()
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  const auto now = this->now();

  for (auto & [rid, ni] : navs_) {
    if (!ni || !ni->chassis_task_sent) continue;

    std::string task_id = ni->current_task_id;

    // 握手超时: 重试或失败
    if (!ni->chassis_handshake_ok) {
      if (now > ni->chassis_hs_deadline) {
        ni->chassis_retries++;
        if (ni->chassis_retries <= chassis_max_retries_) {
          send_chassis_cmd(rid, task_id, ni->pending_wp_num, ni->pending_task_type, ni->pending_site_code);
        } else {
          ni->chassis_task_sent = false;
          ni->chassis_handshake_ok = false;
          ni->chassis_retries = 0;
          ni->chassis_acked = true;
          scheduler_->fail_task(task_id, "chassis handshake timeout");
          fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
          task_pub_->publish(ti);
          finalize_task_completion(rid, task_id);
        }
      }
      continue;
    }

    // 执行超时
    if (now > ni->chassis_exec_deadline) {
      ni->chassis_task_sent = false;
      ni->chassis_handshake_ok = false;
      ni->chassis_acked = true;
      scheduler_->fail_task(task_id, "chassis execution timeout");
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
      task_pub_->publish(ti);
      finalize_task_completion(rid, task_id);
    }
  }
}

bool FleetManagerNode::is_robot_executing(const std::string & robot_id) const
{
  auto it = navs_.find(robot_id);
  return (it != navs_.end() && it->second && it->second->chassis_task_sent);
}

// ============================================================================
// 任务完成清理 — 释放预留 + 清空导航 + 死胡同自动驶离
// ============================================================================

void FleetManagerNode::finalize_task_completion(
  const std::string & robot_id, const std::string & task_id)
{
  auto ni = get_or_create_nav(robot_id);
  if (!ni) return;
  clear_task_wait_condition(task_id);
  occupancy_->release_reservations(robot_id);
  ni->current_task_id.clear();
  ni->has_active_goal = false;
  ni->route.clear();
  ni->route_index = 0;
  ni->route_alignment_done = false;
  ni->retry_count = 0;
  ni->chassis_task_sent = false;
  ni->chassis_handshake_ok = false;
  ni->chassis_retries = 0;

  // auto_relocate 已移除：机器人完成任务后留在目标航点，不自动驶离
}

uint16_t FleetManagerNode::wp_to_u16(const std::string & wp_id) const
{
  std::string nums;
  for (auto it = wp_id.rbegin(); it != wp_id.rend(); ++it) {
    if (std::isdigit(*it)) nums.insert(nums.begin(), *it);
    else if (!nums.empty()) break;
  }
  if (nums.empty()) return static_cast<uint16_t>(std::hash<std::string>{}(wp_id) & 0xFFFF);
  return static_cast<uint16_t>(std::stoul(nums) & 0xFFFF);
}

// ============================================================================
// LED 状态灯
// ============================================================================

uint8_t FleetManagerNode::determine_led_state(const std::string & robot_id) const
{
  auto ni = navs_.find(robot_id);
  if (ni == navs_.end() || !ni->second) return 3;  // 空闲
  if (ni->second->chassis_task_sent) return 2;      // 任务执行中
  if (ni->second->has_active_goal)  return 0;       // 行走中
  bool traffic_wait = false;
  for (const auto & [_, wait] : task_waits_) {
    if (wait.robot_id == robot_id) {
      traffic_wait = true;
      break;
    }
  }
  if (!traffic_wait) {
    for (const auto & [_, waiters] : waiting_for_) {
      if (waiters.count(robot_id)) {
        traffic_wait = true;
        break;
      }
    }
  }
  if (traffic_wait) return 1;                       // 交通等待
  if (!ni->second->route.empty() && !ni->second->current_task_id.empty()) return 0;
  if (!ni->second->current_task_id.empty()) return 1; // 交通等待
  return 3;  // 空闲
}

void FleetManagerNode::led_timer_callback()
{
  for (auto & [rid, ni] : navs_) {
    if (!ni || !ni->led_pub) continue;
    auto st = robots_.find(rid);
    if (st == robots_.end() || st->second.connection_status != "online") continue;

    uint8_t s = determine_led_state(rid);
    fleet_msgs::msg::LEDTask msg;
    msg.state = s;
    ni->led_pub->publish(msg);
    ni->last_led_state = s;
  }
}

void FleetManagerNode::led_status_callback(
  const std::string & robot_id, const fleet_msgs::msg::LEDStatus::SharedPtr msg)
{
  auto ni = get_or_create_nav(robot_id);
  if (!ni) return;
  ni->chassis_led = msg->state;
  ni->led_received = msg->received;
}

// ============================================================================
// 导航工具函数
// ============================================================================

void FleetManagerNode::cancel_goals(const std::shared_ptr<RobotNavInfo> & ni)
{
  if (!ni) return;
  ni->recent_cancel_until = this->now() + rclcpp::Duration::from_seconds(kNavCancelSettlingSec);
  if (ni->goal_handle && ni->nav_client) {
    ni->nav_client->async_cancel_goal(ni->goal_handle);
    ni->goal_handle.reset();
  }
  if (ni->align_goal_handle && ni->align_client) {
    ni->align_client->async_cancel_goal(ni->align_goal_handle);
    ni->align_goal_handle.reset();
  }
  ni->has_active_goal = false;
  ni->aligning_before_nav = false;
  ni->nav_seq++;
}

void FleetManagerNode::cancel_all_goals()
{
  for (auto & [_, ni] : navs_) cancel_goals(ni);
}

void FleetManagerNode::stop_robot(const std::string & robot_id, int burst)
{
  if (vel_pubs_.find(robot_id) == vel_pubs_.end())
    vel_pubs_[robot_id] = this->create_publisher<geometry_msgs::msg::Twist>(
      "/" + robot_id + "/cmd_vel", 10);
  geometry_msgs::msg::Twist zero;
  for (int i = 0; i < burst; ++i) vel_pubs_[robot_id]->publish(zero);
}

void FleetManagerNode::stop_all()
{
  for (auto & [rid, _] : navs_) stop_robot(rid, 5);
}

std::shared_ptr<RobotNavInfo> FleetManagerNode::get_or_create_nav(
  const std::string & robot_id)
{
  auto it = navs_.find(robot_id);
  if (it != navs_.end()) return it->second;

  // 懒初始化: action client + task publishers + LED publishers
  auto ni = std::make_shared<RobotNavInfo>();
  ni->nav_client = rclcpp_action::create_client<NavigateToPose>(
    this, "/" + robot_id + "/navigate_to_pose");
  ni->align_client = rclcpp_action::create_client<NavigateThroughPoses>(
    this, "/" + robot_id + "/navigate_through_poses");
  ni->task_cmd_pub = this->create_publisher<fleet_msgs::msg::TaskCmd>(
    "/" + robot_id + "/task/assign", 10);
  ni->task_fb_sub = this->create_subscription<fleet_msgs::msg::TaskFb>(
    "/" + robot_id + "/task/feedback", 10,
    [this, robot_id](const fleet_msgs::msg::TaskFb::SharedPtr m) {
      chassis_fb_callback(robot_id, m);
    });
  ni->led_pub = this->create_publisher<fleet_msgs::msg::LEDTask>(
    "/" + robot_id + "/led/task", 10);
  ni->led_sub = this->create_subscription<fleet_msgs::msg::LEDStatus>(
    "/" + robot_id + "/led/status", rclcpp::QoS(10).reliable(),
    [this, robot_id](const fleet_msgs::msg::LEDStatus::SharedPtr m) {
      led_status_callback(robot_id, m);
    });

  navs_[robot_id] = ni;
  return ni;
}

double FleetManagerNode::normalize_angle(double a) const
{
  while (a >  M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

double FleetManagerNode::get_yaw(const geometry_msgs::msg::Quaternion & q) const
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double r, p, y;
  tf2::Matrix3x3 m(tq);
  m.getRPY(r, p, y);
  return y;
}

std::string FleetManagerNode::join_route(const std::vector<std::string> & wps) const
{
  return join_waypoints(wps);
}

// ============================================================================
// 链式撤退 — 多车互锁协调
// ============================================================================

bool FleetManagerNode::is_robot_idle(const std::string & robot_id) const
{
  auto ni = navs_.find(robot_id);
  if (ni == navs_.end() || !ni->second) return false;
  return !ni->second->has_active_goal &&
         ni->second->current_task_id.empty() &&
         ni->second->route.empty() &&
         !ni->second->chassis_task_sent;
}

bool FleetManagerNode::is_robot_stationary(const std::string & robot_id) const
{
  auto ni = navs_.find(robot_id);
  if (ni == navs_.end() || !ni->second) return false;
  // 静止 = 无活跃导航 goal 且无底盘任务(不论是否有待处理的用户任务)
  return !ni->second->has_active_goal && !ni->second->chassis_task_sent;
}

bool FleetManagerNode::is_mutual_block(
  const std::string & blocker, const std::string & blocker_wp,
  const std::string &, const std::string & requester_wp) const
{
  // 检查阻塞者的所有出口是否都必经请求者的位置
  auto conns = traffic_->get_waypoint_connections(blocker_wp);
  for (const auto & nb : conns) {
    if (nb == requester_wp) continue;
    std::string b = occupancy_->can_enter(blocker, blocker_wp, nb);
    if (b.empty() || b == blocker) return false;  // 有独立出口 → 非互锁
  }
  return true;  // 全堵塞 → 互锁
}

// ============================================================================
// 递归推占据者: 尝试把 wp 上的机器人推到其分支空闲航点。
// 如果所有分支都被占据，递归推分支上的占据者。
// 返回 true 时 steps 中包含从内到外的推让步骤。
// ============================================================================

bool FleetManagerNode::try_push_occupant(
  const std::string & wp,
  const std::set<std::string> & excluded,
  std::set<std::string> & visited,
  int depth,
  std::vector<RetreatChainStep> & steps)
{
  if (depth >= 4) return false;

  auto holder = occupancy_->get_zone_holder(wp);
  if (holder.empty()) return true;
  if (!is_robot_stationary(holder)) return false;
  // 顶层(非递归)检查已访问机器人防环路
  if (depth == 0 && visited.count(holder)) return false;
  visited.insert(holder);

  auto conns = traffic_->get_waypoint_connections(wp);
  for (const auto & hc : conns) {
    if (excluded.count(hc)) continue;

    auto hc_holder = occupancy_->get_zone_holder(hc);
    if (hc_holder.empty() || hc_holder == holder) {
      steps.push_back({holder, hc});
      PersistLogger::log_info("nav.push_occupant", holder, "",
        "pushing " + holder + " from " + wp + " to " + hc + " depth=" + std::to_string(depth),
        __FILE__, __LINE__, __func__);
      return true;
    }

    // 递归推占据者(用深度限制防无限递归,允许跨层 swap: A→B, B→A)
    if (is_robot_stationary(hc_holder)) {
      std::vector<RetreatChainStep> sub_steps;
      if (try_push_occupant(hc, excluded, visited, depth + 1, sub_steps)) {
        for (auto & s : sub_steps) steps.push_back(std::move(s));
        steps.push_back({holder, hc});
        PersistLogger::log_info("nav.push_occupant", holder, "",
          "pushing " + holder + " from " + wp + " to " + hc + " depth=" + std::to_string(depth) +
          " (after pushing " + hc_holder + ")",
          __FILE__, __LINE__, __func__);
        return true;
      }
    }
  }
  return false;
}

bool FleetManagerNode::try_build_retreat_chain(
  const std::string & requester, const std::string & from_wp,
  const std::string & to_wp, const std::string & blocker,
  const std::set<std::string> & blocked_set, int depth)
{
  if (depth >= kMaxChainDepth) return false;

  chain_plan_.steps.clear();

  auto conns = traffic_->get_waypoint_connections(from_wp);
  std::string best_retreat;
  std::vector<RetreatChainStep> best_subchain;

  // 搜索请求者的撤退方向(优先直接空闲航点, 其次请求占据者让路)
  for (const auto & nb : conns) {
    if (nb == to_wp || blocked_set.count(nb)) continue;
    auto direct_blocker = occupancy_->can_enter(requester, from_wp, nb);

    auto holder = occupancy_->get_zone_holder(nb);
    if (!direct_blocker.empty() && direct_blocker != requester) {
      if (holder.empty() || holder != direct_blocker || !is_robot_stationary(holder)) continue;
    }
    if (holder.empty() || holder == requester) {
      best_retreat = nb;
      best_subchain.clear();
      break;  // 直接空闲 → 最优
    }

    // 被占据 → 递归推占据者让路
    if (is_robot_stationary(holder)) {
      std::set<std::string> push_excluded = blocked_set;
      push_excluded.insert(from_wp);
      push_excluded.insert(to_wp);
      std::set<std::string> push_visited;
      std::vector<RetreatChainStep> push_steps;
      if (try_push_occupant(nb, push_excluded, push_visited, 0, push_steps)) {
        best_subchain = std::move(push_steps);
        best_retreat = nb;
        break;
      }
    }
  }

  // 组装链: [清路(请求者侧)...] → [请求者撤退] → [清路(阻塞者侧)...] → [阻塞者退出] → [请求者恢复]
  for (auto & s : best_subchain) chain_plan_.steps.push_back(std::move(s));

  if (!best_retreat.empty()) {
    chain_plan_.steps.push_back({requester, best_retreat});
  } else {
    PersistLogger::log_info("nav.chain_requester_hold", requester, "",
      "requester has no safe retreat from " + from_wp + ", moving blocker first",
      __FILE__, __LINE__, __func__);
  }

  // 为阻塞者找退出目标(非阻塞航点, 非撤退航点, 非请求者路径上的航点)
  // 如果直接邻居被占据，也尝试推占据者让路
  std::string blocker_dest;
  std::vector<RetreatChainStep> blocker_clear_steps;
  std::set<std::string> requester_future_wps;
  auto requester_ni_it = navs_.find(requester);
  if (requester_ni_it != navs_.end() && requester_ni_it->second) {
    const auto & route = requester_ni_it->second->route;
    auto to_it = std::find(route.begin(), route.end(), to_wp);
    if (to_it != route.end()) {
      for (auto it = std::next(to_it); it != route.end(); ++it) {
        requester_future_wps.insert(*it);
      }
    }
  }
  for (const auto & nb : traffic_->get_waypoint_connections(to_wp)) {
    if (nb == from_wp || (!best_retreat.empty() && nb == best_retreat)) continue;
    bool reserved_future = requester_future_wps.count(nb) > 0;
    if (reserved_future || blocked_set.count(nb)) continue;
    auto direct_blocker = occupancy_->can_enter(blocker, to_wp, nb);
    auto nb_holder = occupancy_->get_zone_holder(nb);
    if (!direct_blocker.empty() && direct_blocker != blocker) {
      if (nb_holder.empty() || nb_holder != direct_blocker || !is_robot_stationary(nb_holder)) continue;
    }
    if (nb_holder.empty()) { blocker_dest = nb; break; }
    // 被占据 → 递归推占据者让路
    if (is_robot_stationary(nb_holder)) {
      std::set<std::string> push_excluded = blocked_set;
      push_excluded.insert(from_wp);
      if (!best_retreat.empty()) push_excluded.insert(best_retreat);
      push_excluded.insert(to_wp);
      std::set<std::string> push_visited;
      if (try_push_occupant(nb, push_excluded, push_visited, 0, blocker_clear_steps)) {
        blocker_dest = nb;
        break;
      }
    }
  }
  if (blocker_dest.empty()) {
    std::vector<std::string> best_exit_path;
    std::vector<RetreatChainStep> best_exit_clear_steps;
    double best_exit_cost = std::numeric_limits<double>::max();
    auto all_wps = traffic_->get_all_waypoint_poses();
    auto from_pose = traffic_->get_waypoint_pose(to_wp);
    for (const auto & [candidate, pose] : all_wps) {
      if (candidate == to_wp || candidate == from_wp ||
          (!best_retreat.empty() && candidate == best_retreat) ||
          blocked_set.count(candidate) || requester_future_wps.count(candidate)) {
        continue;
      }
      auto candidate_holder = occupancy_->get_zone_holder(candidate);
      bool candidate_needs_clear = !candidate_holder.empty() && candidate_holder != blocker;
      if (candidate_needs_clear && !is_robot_stationary(candidate_holder)) continue;
      if (!candidate_needs_clear && !occupancy_->is_zone_free_for(blocker, candidate)) continue;
      auto path = traffic_->find_path(to_wp, candidate);
      if (path.size() < 2) continue;
      bool blocked_path = false;
      for (size_t i = 1; i < path.size(); ++i) {
        if (path[i] == from_wp || (!best_retreat.empty() && path[i] == best_retreat)) {
          blocked_path = true;
          break;
        }
        if (path[i] != candidate && blocked_set.count(path[i]) &&
            requester_future_wps.count(path[i]) == 0) {
          blocked_path = true;
          break;
        }
        auto hop_blocker = occupancy_->can_enter(blocker, path[i - 1], path[i]);
        if (!hop_blocker.empty() && hop_blocker != blocker) {
          if (path[i] == candidate && candidate_needs_clear &&
              hop_blocker == candidate_holder) {
            continue;
          }
          blocked_path = true;
          break;
        }
      }
      if (blocked_path) continue;
      std::vector<RetreatChainStep> candidate_clear_steps;
      if (candidate_needs_clear) {
        std::set<std::string> push_excluded = blocked_set;
        push_excluded.insert(from_wp);
        push_excluded.insert(to_wp);
        if (!best_retreat.empty()) push_excluded.insert(best_retreat);
        for (const auto & wp : requester_future_wps) push_excluded.insert(wp);
        std::set<std::string> push_visited;
        if (!try_push_occupant(candidate, push_excluded, push_visited, 0, candidate_clear_steps)) {
          continue;
        }
      }
      const double dist = std::hypot(
        pose.position.x - from_pose.position.x,
        pose.position.y - from_pose.position.y);
      const double cost = static_cast<double>(path.size() + candidate_clear_steps.size()) * 10.0 + dist;
      if (cost < best_exit_cost) {
        best_exit_cost = cost;
        best_exit_path = path;
        best_exit_clear_steps = std::move(candidate_clear_steps);
      }
    }
    if (!best_exit_path.empty()) {
      blocker_dest = best_exit_path.back();
      blocker_clear_steps = std::move(best_exit_clear_steps);
    }
    if (blocker_dest.empty()) {
      PersistLogger::log_warn("nav.chain_fallback_empty", requester, "",
        "from=" + from_wp + " best_retreat=" + best_retreat + " to_wp=" + to_wp,
        __FILE__, __LINE__, __func__);
    }
  }
  if (blocker_dest.empty()) {
    PersistLogger::log_warn("nav.chain_no_blocker_exit", requester, "",
      "cannot find blocker exit from " + to_wp + " (requester_from=" + from_wp +
      " best_retreat=" + best_retreat + ")",
      __FILE__, __LINE__, __func__);
    chain_plan_.steps.clear();
    return false;
  }
  // 先插入阻塞者清路步骤，再插入阻塞者退出步骤
  for (auto & s : blocker_clear_steps) chain_plan_.steps.push_back(std::move(s));

  chain_plan_.steps.push_back({blocker, blocker_dest});

  // 请求者恢复到原目标
  chain_plan_.steps.push_back({requester, to_wp});

  return true;
}

// ============================================================================
// 启动 target 退让 chain retreat (公用入口)
//   将 target_holder 通过 chain push 推出 target_wp,腾出空间给 requester。
//   调用方须已确认 chain_plan_.active==false。成功返回 true。
// ============================================================================

bool FleetManagerNode::try_start_target_exit_chain(
  const std::string & target_holder,
  const std::string & target_wp,
  const std::string & requester_rid,
  const std::string & requester_task_id,
  const std::set<std::string> & exclude)
{
  if (chain_plan_.active) return false;
  if (target_holder.empty() || target_wp.empty()) return false;

  std::vector<RetreatChainStep> clear_steps;
  std::string chain_exit_wp;
  for (const auto & nb : traffic_->get_waypoint_connections(target_wp)) {
    if (exclude.count(nb)) continue;
    std::string blocker = occupancy_->can_enter(target_holder, target_wp, nb);
    if (blocker.empty() || blocker == target_holder) {
      chain_exit_wp = nb;
      break;
    }
    auto nb_holder = occupancy_->get_zone_holder(nb);
    if (nb_holder.empty() || !is_robot_stationary(nb_holder)) continue;
    std::set<std::string> push_excluded = exclude;
    push_excluded.insert(target_wp);
    std::set<std::string> visited;
    std::vector<RetreatChainStep> push_steps;
    if (try_push_occupant(nb, push_excluded, visited, 0, push_steps)) {
      clear_steps = std::move(push_steps);
      chain_exit_wp = nb;
      break;
    }
  }
  if (chain_exit_wp.empty()) return false;

  const auto now = this->now();
  chain_plan_.steps.clear();
  chain_plan_.saved_task_ids.clear();
  chain_plan_.saved_targets.clear();
  for (auto & s : clear_steps) chain_plan_.steps.push_back(std::move(s));
  chain_plan_.steps.push_back({target_holder, chain_exit_wp});
  for (const auto & s : chain_plan_.steps) {
    auto ri = get_or_create_nav(s.robot_id);
    if (!ri || ri->current_task_id.empty()) continue;
    chain_plan_.saved_task_ids[s.robot_id] = ri->current_task_id;
    if (!ri->route.empty()) chain_plan_.saved_targets[s.robot_id] = ri->route.back();
    ri->has_active_goal = false;
    ri->route.clear();
    ri->route_index = 0;
    ri->retry_count = 0;
  }
  chain_plan_.original_requester = requester_rid;
  chain_plan_.original_target = target_wp;
  chain_plan_.original_task_id = requester_task_id;
  chain_plan_.active = true;
  chain_plan_.started_at = now;
  chain_plan_.step_started_at = now;
  chain_plan_.current_step = 0;
  chain_plan_.step_retry_count = 0;
  PersistLogger::log_info("sched.target_exit_chain_started", target_holder, requester_task_id,
    "moving target holder from " + target_wp + " to " + chain_exit_wp +
    " with " + std::to_string(chain_plan_.steps.size()) + " internal steps",
    __FILE__, __LINE__, __func__);
  execute_chain_step();
  // 注: 调用方负责 mark_snapshot_dirty (本函数定义在 navigation cpp,无 lambda 上下文)
  return true;
}

// ============================================================================
// 强制 blocker yield (#2 自愈)
//   当一个机器人长期 hold 自身任务以让位 active waiters 但 hold 反复触发时,
//   主动给它派 avoidance 退到附近安全 wp,避免被动死等。
//   exclude: 自身任务 target、其他 active 任务 target、所有等待者关心的 wp。
//   返回 true 表示成功派出 avoidance,调用方应将自己 hold 的 task 重新入 pending。
// ============================================================================

bool FleetManagerNode::force_blocker_yield(
  const std::string & rid,
  const std::string & current_target_wp,
  const std::string & current_task_id)
{
  auto ni = get_or_create_nav(rid);
  if (!ni) return false;
  // 已经有活跃 nav 或底盘任务中,不能强制 yield
  if (ni->has_active_goal || ni->chassis_task_sent ||
      !ni->route.empty() || !ni->current_task_id.empty()) {
    return false;
  }

  auto rs = robots_.find(rid);
  if (rs == robots_.end()) return false;
  std::string current_wp = rs->second.current_waypoint;
  if (current_wp.empty()) {
    current_wp = traffic_->find_nearest_waypoint(rs->second.current_pose);
  }
  if (current_wp.empty()) return false;

  std::set<std::string> exclude;
  exclude.insert(current_wp);
  if (!current_target_wp.empty()) exclude.insert(current_target_wp);

  // 其他 active 任务 target
  for (const auto & existing : scheduler_->get_all_tasks()) {
    if (existing.task_id.empty() || existing.task_id == current_task_id) continue;
    if (existing.waypoint_id.empty()) continue;
    if (existing.status != "in_progress" &&
        existing.status != "executing" &&
        existing.status != "assigned") {
      continue;
    }
    exclude.insert(existing.waypoint_id);
  }

  // 所有以 rid 为 blocker 的等待者关心的 wp
  size_t waiters_relieved = 0;
  auto snapshot = build_fleet_state_snapshot();
  for (const auto & edge : snapshot.wait_edges) {
    if (edge.blocker_robot_id != rid) continue;
    waiters_relieved++;
    if (!edge.target_wp.empty()) exclude.insert(edge.target_wp);
    if (!edge.to_wp.empty()) exclude.insert(edge.to_wp);
    if (!edge.from_wp.empty()) exclude.insert(edge.from_wp);
  }
  if (waiters_relieved == 0) return false;  // 没有等待者就不需要 yield

  std::string yield_wp = find_safe_free_waypoint(current_wp, exclude, rid);
  if (yield_wp.empty() || yield_wp == current_wp) return false;

  auto yield_path = traffic_->find_path(current_wp, yield_wp);
  if (yield_path.size() < 2) return false;

  ni->route = yield_path;
  ni->route_index = 0;
  ni->route_alignment_done = false;
  ni->current_task_id = "avoidance_" + rid;
  ni->retry_count = 0;
  ni->retry_after = rclcpp::Time{};
  ni->has_active_goal = false;
  PersistLogger::log_info("sched.blocker_force_yield", rid, current_task_id,
    "moving from " + current_wp + " to " + yield_wp +
    " to release " + std::to_string(waiters_relieved) + " active waiters",
    __FILE__, __LINE__, __func__);
  navigate_to_next_waypoint(rid);
  // 注: 调用方负责 mark_snapshot_dirty
  return true;
}

void FleetManagerNode::execute_chain_step()
{
  if (!chain_plan_.active || chain_plan_.current_step >= chain_plan_.steps.size()) return;

  const auto & step = chain_plan_.steps[chain_plan_.current_step];
  chain_plan_.step_started_at = this->now();
  auto ni = get_or_create_nav(step.robot_id);
  if (!ni) { abort_chain("robot nav info missing"); return; }

  auto st = robots_.find(step.robot_id);
  if (st == robots_.end() || st->second.connection_status != "online") {
    abort_chain("robot offline"); return;
  }

  // 步预验证: 目标航点未被意外占用
  std::string wp_holder = occupancy_->get_zone_holder(step.target_wp);
  if (!wp_holder.empty() && wp_holder != step.robot_id) {
    abort_chain("step target occupied by " + wp_holder); return;
  }

  // 验证目标航点存在
  auto all_wps = traffic_->get_all_waypoint_poses();
  if (all_wps.find(step.target_wp) == all_wps.end()) {
    abort_chain("target waypoint not found: " + step.target_wp); return;
  }

  if (ni->has_active_goal || ni->goal_handle || ni->align_goal_handle || ni->aligning_before_nav) {
    cancel_goals(ni);
  } else {
    ni->nav_seq++;
  }

  ni->current_task_id = std::string(kChainTaskPrefix) + step.robot_id;
  ni->retry_count = 0;
  ni->retry_after = rclcpp::Time{};

  // 构建从当前位置到目标航点的完整路径，逐航点导航（不再跳点）
  std::string cur_wp;
  auto st2 = robots_.find(step.robot_id);
  if (st2 != robots_.end()) {
    cur_wp = st2->second.current_waypoint;
    if (cur_wp.empty())
      cur_wp = traffic_->find_nearest_waypoint(st2->second.current_pose);
  }

  std::vector<std::string> full_path;
  if (!cur_wp.empty() && cur_wp != step.target_wp) {
    full_path = traffic_->find_path(cur_wp, step.target_wp);
  }
  if (full_path.empty()) full_path.push_back(step.target_wp);

  // 去重
  std::vector<std::string> clean;
  for (const auto & w : full_path) {
    if (clean.empty() || clean.back() != w) clean.push_back(w);
  }
  ni->route = clean;
  ni->route_index = 0;
  ni->route_alignment_done = false;

  PersistLogger::log_info("nav.chain_step", step.robot_id, ni->current_task_id,
    "step " + std::to_string(chain_plan_.current_step + 1) + "/" +
    std::to_string(chain_plan_.steps.size()) + " target=" + step.target_wp +
    " path=" + join_waypoints(clean),
    __FILE__, __LINE__, __func__);

  navigate_to_next_waypoint(step.robot_id);
}

void FleetManagerNode::on_chain_step_complete(const std::string & robot_id, bool nav_success)
{
  if (!chain_plan_.active) return;

  const auto & step = chain_plan_.steps[chain_plan_.current_step];
  if (step.robot_id != robot_id) return;

  if (!nav_success) {
    chain_plan_.step_retry_count++;
    if (chain_plan_.step_retry_count <= kMaxChainStepRetries) {
      execute_chain_step();
      return;
    }
    abort_chain("step nav failed after " + std::to_string(kMaxChainStepRetries) + " retries");
    return;
  }

  // 清理当前步骤完成后的 nav 状态
  auto ni = get_or_create_nav(robot_id);
  if (ni) {
    occupancy_->release_reservations(robot_id);
    ni->current_task_id.clear();
    ni->has_active_goal = false;
    ni->route.clear();
    ni->route_index = 0;
  }

  chain_plan_.step_retry_count = 0;
  chain_plan_.current_step++;
  chain_plan_.step_started_at = this->now();

  if (chain_plan_.current_step >= chain_plan_.steps.size()) {
    // 链完成 → 恢复所有参与底盘的原始任务
    PersistLogger::log_info("nav.chain_complete", chain_plan_.original_requester,
      chain_plan_.original_task_id,
      "chain retreat finished, resuming " + std::to_string(chain_plan_.saved_task_ids.size()) + " tasks",
      __FILE__, __LINE__, __func__);

    auto saved_tasks = std::move(chain_plan_.saved_task_ids);
    auto saved_targets = std::move(chain_plan_.saved_targets);
    chain_plan_.active = false;
    chain_plan_.steps.clear();
    chain_plan_.current_step = 0;
    chain_plan_.step_started_at = rclcpp::Time{};

    for (const auto & [rid, tid] : saved_tasks) {
      auto ri = get_or_create_nav(rid);
      if (!ri) continue;
      auto rst = robots_.find(rid);
      if (rst == robots_.end() || rst->second.connection_status != "online") continue;

      // 跳过链执行期间被取消的任务
      auto ti_check = scheduler_->get_task_info(tid);
      if (ti_check.task_id.empty() || ti_check.status == "cancelled") {
        PersistLogger::log_info("nav.chain_skip_cancelled", rid, tid,
          "task was cancelled during chain, skipping restore",
          __FILE__, __LINE__, __func__);
        finalize_task_completion(rid, tid);
        continue;
      }

      auto tgt_it = saved_targets.find(rid);
      std::string tgt = (tgt_it != saved_targets.end()) ? tgt_it->second : "";
      if (tgt.empty()) continue;

      auto target_pose = traffic_->get_waypoint_pose(tgt);
      double dx = rst->second.current_pose.position.x - target_pose.position.x;
      double dy = rst->second.current_pose.position.y - target_pose.position.y;
      if (std::hypot(dx, dy) <= waypoint_radius_) {
        scheduler_->complete_task(tid);
        fleet_msgs::msg::TaskInfo pub_ti = scheduler_->get_task_info(tid);
        if (!pub_ti.task_id.empty()) task_pub_->publish(pub_ti);
        finalize_task_completion(rid, tid);
      } else {
        start_navigation(rid, tgt, tid);
      }
    }
  } else {
    execute_chain_step();
  }
}

void FleetManagerNode::abort_chain(const std::string & reason)
{
  if (!chain_plan_.active) return;

  PersistLogger::log_warn("nav.chain_abort", chain_plan_.original_requester,
    chain_plan_.original_task_id,
    "chain aborted: " + reason,
    __FILE__, __LINE__, __func__);

  // 清理所有链参与底盘的导航状态
  for (size_t i = chain_plan_.current_step; i < chain_plan_.steps.size(); ++i) {
    const auto & step = chain_plan_.steps[i];
    auto ni = get_or_create_nav(step.robot_id);
    if (ni) {
      cancel_goals(ni);
      occupancy_->release_reservations(step.robot_id);
      stop_robot(step.robot_id, 5);
      ni->current_task_id.clear();
      ni->has_active_goal = false;
      ni->route.clear();
      ni->route_index = 0;
    }
  }

  auto saved_tasks = std::move(chain_plan_.saved_task_ids);
  chain_plan_.active = false;
  chain_plan_.steps.clear();
  chain_plan_.current_step = 0;
  chain_plan_.step_started_at = rclcpp::Time{};

  // 恢复所有已保存任务;非 fixed 任务释放旧绑定避免反复回到同一瓶颈。
  // 注: chain abort 是协调层失败,不是 task 本身失败,因此用 mark_task_pending
  // 不增 retry_cycle_count_;否则反复 chain abort 会误杀被动参与者。
  // 任务在下一 tick 即被 assign_pending_tasks 拾起,等价于"立即重试"。
  for (const auto & [rid, tid] : saved_tasks) {
    if (tid.empty()) continue;
    scheduler_->mark_task_pending(tid);
    fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
    if (!ti.task_id.empty()) task_pub_->publish(ti);
  }
}

}  // namespace fleet_manager
