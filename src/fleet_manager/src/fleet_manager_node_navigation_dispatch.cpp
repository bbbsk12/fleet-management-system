#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#include <tf2/LinearMath/Quaternion.hpp>

namespace fleet_manager
{
namespace
{
constexpr double kNavControlCommitSec = 0.9;
}

void FleetManagerNode::navigate_through_waypoints(
  const std::string & robot_id,
  const std::vector<std::string> & waypoint_path)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info || waypoint_path.empty()) return;

  // 统一单点导航：将路径回填到 route，并继续走 navigate_to_next_waypoint。
  reset_through_segment_state(nav_info);
  nav_info->route_waypoints = waypoint_path;
  nav_info->current_waypoint_index = 0;
  navigate_to_next_waypoint(robot_id);
}

void FleetManagerNode::navigate_to_waypoint(
  const std::string & robot_id,
  const std::string & waypoint_id,
  const std::string & task_id,
  bool is_final_waypoint)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info) {
    if (!is_internal_task_id(task_id)) task_scheduler_->fail_task(task_id, "No nav client");
    return;
  }

  const auto now = this->now();
  if (nav_info->has_active_goal &&
      nav_info->nav_control_commit_until.nanoseconds() > 0 &&
      now < nav_info->nav_control_commit_until)
  {
    nav_info->pending_waypoint_path = {waypoint_id};
    PersistLogger::log_info(
      "nav.commit_defer", robot_id, task_id,
      "defer waypoint dispatch during commit window waypoint=" + waypoint_id,
      __FILE__, __LINE__, __func__);
    return;
  }
  if (nav_info->recent_cancel_until.nanoseconds() > 0 && now < nav_info->recent_cancel_until) {
    auto retry_path = nav_info->route_waypoints;
    if (retry_path.empty()) {
      std::string current_wp;
      auto robot_it = robot_statuses_.find(robot_id);
      if (robot_it != robot_statuses_.end() && robot_it->second.location_type == "waypoint") {
        current_wp = robot_it->second.current_waypoint;
      }
      if (!current_wp.empty() && current_wp != waypoint_id) {
        retry_path = {current_wp, waypoint_id};
      } else {
        retry_path = {waypoint_id};
      }
    }
    nav_info->pending_waypoint_path = retry_path;
    PersistLogger::log_info(
      "nav.defer", robot_id, task_id,
      "recent cancel settling, defer waypoint=" + waypoint_id,
      __FILE__, __LINE__, __func__);
    return;
  }

  cancel_active_robot_goals(nav_info);
  const uint64_t cmd_seq = ++nav_info->nav_command_seq;

  // 非阻塞检查 action server 可用性（单点导航仅依赖 NavigateToPose）
  const bool pose_ready = action_client_ready<NavigateToPose>(nav_info->nav_client);
  if (!nav_info->nav_client) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "event=nav.defer task=%s robot=%s state_prev=in_progress state_new=in_progress reason=NAV_SERVER_NOT_READY detail=cache_single_waypoint",
      task_id.c_str(), robot_id.c_str());
    nav_info->pending_waypoint_path = {waypoint_id};
    return;
  }
  if (!pose_ready) {
    PersistLogger::log_info(
      "nav.mode_fallback", robot_id, task_id,
      "NavigateToPose readiness false, dispatching optimistically to " + waypoint_id,
      __FILE__, __LINE__, __func__);
  }

  geometry_msgs::msg::Pose waypoint_pose = traffic_manager_->get_waypoint_pose(waypoint_id);

  // 朝向
  auto robot_it = robot_statuses_.find(robot_id);
  bool keep_current_orientation = false;
  double target_yaw = 0.0;
  if (robot_it != robot_statuses_.end()) {
    const double dx_to_goal = waypoint_pose.position.x - robot_it->second.current_pose.position.x;
    const double dy_to_goal = waypoint_pose.position.y - robot_it->second.current_pose.position.y;
    const double dist_to_goal = std::sqrt(dx_to_goal * dx_to_goal + dy_to_goal * dy_to_goal);
    keep_current_orientation = is_final_waypoint && dist_to_goal <= final_orientation_relax_distance_;
  }

  if (keep_current_orientation && robot_it != robot_statuses_.end()) {
    waypoint_pose.orientation = robot_it->second.current_pose.orientation;
  } else {
    if (is_final_waypoint && nav_info->route_waypoints.size() >= 2) {
      size_t prev_idx = (nav_info->current_waypoint_index > 0) ? nav_info->current_waypoint_index - 1 : 0;
      auto prev_pose = traffic_manager_->get_waypoint_pose(nav_info->route_waypoints[prev_idx]);
      target_yaw = std::atan2(waypoint_pose.position.y - prev_pose.position.y,
                              waypoint_pose.position.x - prev_pose.position.x);
    } else if (robot_it != robot_statuses_.end()) {
      target_yaw = std::atan2(waypoint_pose.position.y - robot_it->second.current_pose.position.y,
                              waypoint_pose.position.x - robot_it->second.current_pose.position.x);
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, target_yaw);
    waypoint_pose.orientation.x = q.x();
    waypoint_pose.orientation.y = q.y();
    waypoint_pose.orientation.z = q.z();
    waypoint_pose.orientation.w = q.w();
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.pose = waypoint_pose;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this, robot_id, cmd_seq](GoalHandleNavigateToPose::SharedPtr goal_handle) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      auto ni = get_robot_nav_info(robot_id);
      if (!ni || ni->nav_command_seq != cmd_seq) return;
      if (!goal_handle) {
        RCLCPP_ERROR(
          this->get_logger(),
          "event=nav.goal_rejected task=%s robot=%s state_prev=in_progress state_new=failed reason=NAVIGATE_TO_POSE_REJECTED",
          ni->current_task_id.empty() ? "-" : ni->current_task_id.c_str(),
          robot_id.c_str());
        ni->has_active_goal = false;
      } else {
        ni->current_pose_goal_handle = goal_handle;
      }
    };

  send_goal_options.feedback_callback =
    [this, robot_id, cmd_seq](GoalHandleNavigateToPose::SharedPtr,
                     const std::shared_ptr<const NavigateToPose::Feedback>) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      auto ni = get_robot_nav_info(robot_id);
      if (!ni || ni->nav_command_seq != cmd_seq) return;
      ni->nav_last_activity = this->now();
      if (!ni->current_task_id.empty() && !is_internal_task_id(ni->current_task_id)) {
        fleet_msgs::msg::TaskInfo ti;
        ti.task_id = ni->current_task_id;
        ti.status = "in_progress";
        task_status_pub_->publish(ti);
      }
    };

  send_goal_options.result_callback =
    [this, robot_id, cmd_seq](const GoalHandleNavigateToPose::WrappedResult & result) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      auto ni = get_robot_nav_info(robot_id);
      if (!ni || ni->nav_command_seq != cmd_seq) {
        PersistLogger::log_warn("nav.result_seq_mismatch", robot_id,
          ni ? ni->current_task_id : "",
          "NavigateToPose result dropped: cmd_seq=" + std::to_string(cmd_seq) +
          " current_seq=" + std::to_string(ni ? ni->nav_command_seq : 0) +
          " code=" + std::to_string(static_cast<int>(result.code)),
          __FILE__, __LINE__, __func__);
        return;
      }
      ni->has_active_goal = false;
      ni->current_pose_goal_handle.reset();

      const std::string tid = ni->current_task_id;
      if (tid.empty()) return;
      const bool internal = is_internal_task_id(tid);
      PersistLogger::log_info("nav.result", robot_id, tid,
        "NavigateToPose result code=" + std::to_string(static_cast<int>(result.code)),
        __FILE__, __LINE__, __func__);
      // Hold cancel 保护
      if (result.code == rclcpp_action::ResultCode::CANCELED) {
        if (hold_contexts_.find(robot_id) != hold_contexts_.end()) {
          PersistLogger::log_info("nav.result", robot_id, tid,
            "NavigateToPose CANCELED during hold, skipping fail", __FILE__, __LINE__, __func__);
          return;
        }
      }
      // SUCCEEDED 时检查是否到终点——中间航点不完成任务
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        if (!ni->route_waypoints.empty() &&
            ni->current_waypoint_index + 1 < ni->route_waypoints.size()) {
          PersistLogger::log_info("nav.result", robot_id, tid,
            "NavigateToPose reached intermediate wp idx=" +
            std::to_string(ni->current_waypoint_index) + ", continuing route",
            __FILE__, __LINE__, __func__);
          navigate_to_next_waypoint(robot_id);
          return;
        }
      }
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          if (!internal) on_navigation_succeeded(robot_id, tid);
          break;
        case rclcpp_action::ResultCode::ABORTED:
          if (!internal) { task_scheduler_->fail_task(tid, "Navigation aborted"); finalize_task_completion(robot_id, tid); }
          break;
        case rclcpp_action::ResultCode::CANCELED:
          if (!internal) {
            const auto before = task_scheduler_->get_task_info(tid);
            if (before.task_id.empty() || before.status != "cancelled") {
              task_scheduler_->fail_task(tid, "Navigation canceled");
            }
            finalize_task_completion(robot_id, tid);
          }
          break;
        default:
          if (!internal) { task_scheduler_->fail_task(tid, "Unknown result"); finalize_task_completion(robot_id, tid); }
          break;
      }
      occupancy_manager_->release_reservations(robot_id);
      // Round 12c: 避让任务成功完成后，导航回原始位置
      // Round 12c-fix: 回归前检查是否有机器人 hold 等待回归航点，避免回归后重新阻塞
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && internal) {
        auto ret_it = avoidance_return_waypoint_.find(robot_id);
        if (ret_it != avoidance_return_waypoint_.end()) {
          std::string return_wp = ret_it->second;
          // 若本机器人有未激活的 staged hold（如 dead_end_requester_retreat），
          // 说明 stage_safe_hold 正等待机器人到达退让点后再激活 hold。
          // 此时不可启动回归导航——否则回归的 Nav2 目标会在 hold 激活前发出，
          // hold 的 cancel 来不及拦截，导致机器人在 hold 状态下继续行驶。
          {
            auto self_hold_it = hold_contexts_.find(robot_id);
            if (self_hold_it != hold_contexts_.end() && !self_hold_it->second.active) {
              PersistLogger::log_info("avoidance.return_skip_staged_hold", robot_id, "",
                "skip return to " + return_wp + ": staged hold pending (reason=" +
                self_hold_it->second.reason + ")",
                __FILE__, __LINE__, __func__);
              avoidance_return_waypoint_.erase(ret_it);
              // 重要：即使跳过回归，也必须清理本次内部避让导航状态，
              // 否则会残留 internal task/route，导致后续调度被错误阻塞。
              if (ni) {
                ni->route_waypoints.clear();
                ni->current_waypoint_index = 0;
                ni->current_task_id.clear();
                ni->has_active_goal = false;
                ni->pre_rotate_pending = false;
                ni->pending_waypoint_path.clear();
              }
              return;
            }
          }
          // 检查是否有机器人 hold 等待回归目的地——如果有，放弃回归
          bool return_blocked = false;
          bool system_settling = false;
          for (const auto & [hid, hctx] : hold_contexts_) {
            if (hctx.active || !hctx.pending_hold_waypoint.empty()) {
              system_settling = true;
              PersistLogger::log_info("avoidance.return_skip_system_settling", robot_id, "",
                "skip return to " + return_wp + ": hold flow still settling on " + hid,
                __FILE__, __LINE__, __func__);
              break;
            }
          }
          for (const auto & [hid, hctx] : hold_contexts_) {
            if (hctx.active && hctx.release_wait_waypoint == return_wp) {
              return_blocked = true;
              PersistLogger::log_info("avoidance.return_blocked", robot_id, "",
                "skip return to " + return_wp + ": " + hid +
                " is held waiting for that waypoint",
                __FILE__, __LINE__, __func__);
              break;
            }
          }
          avoidance_return_waypoint_.erase(ret_it);
          // 避让任务完成，重置避让代数
          robot_avoidance_generation_.erase(robot_id);
          const bool auto_return_enabled = false;
          if (!auto_return_enabled) {
            // Convergence-first policy: keep vehicle at avoidance waypoint.
            PersistLogger::log_info("avoidance.return_disabled", robot_id, "",
              "skip auto-return to " + return_wp + " to prevent re-blocking",
              __FILE__, __LINE__, __func__);
            return;
          }
          if (!return_blocked && !system_settling) {
            auto st_it2 = robot_statuses_.find(robot_id);
            std::string cur_wp;
            if (st_it2 != robot_statuses_.end()) {
              cur_wp = st_it2->second.current_waypoint;
              if (cur_wp.empty())
                cur_wp = traffic_manager_->find_nearest_waypoint(st_it2->second.current_pose, 2.5);
            }
            if (!cur_wp.empty() && cur_wp != return_wp) {
              auto return_path = traffic_manager_->find_path(cur_wp, return_wp);
              if (!return_path.empty()) {
                // Round 17+19: 检查回归路径的中间/终点是否与其他机器人已规划路线有航点重叠
                // 防止回归途中与对方在共享航点（如 wp_007）物理汇聚
                // R19: 也检查处于 HOLD 状态但有规划路线的机器人
                bool return_route_conflict = false;
                for (const auto & [oid, onav] : robot_nav_info_) {
                  if (oid == robot_id) continue;
                  if (onav->route_waypoints.size() < 2) continue;
                  // R19: 不再仅检查 has_active_goal；HOLD 中的机器人也有已规划路线
                  if (!onav->has_active_goal && onav->current_task_id.empty()) continue;
                  for (size_t ri = 1; ri < return_path.size() && !return_route_conflict; ++ri) {
                    for (size_t oi = 0; oi < onav->route_waypoints.size(); ++oi) {
                      if (return_path[ri] == onav->route_waypoints[oi]) {
                        return_route_conflict = true;
                        PersistLogger::log_info("avoidance.return_route_conflict", robot_id, "",
                          "return to " + return_wp + " via " + return_path[ri] +
                          " conflicts with " + oid + " route wp " +
                          onav->route_waypoints[oi] + ", skip return",
                          __FILE__, __LINE__, __func__);
                        break;
                      }
                    }
                  }
                  if (return_route_conflict) break;
                }
                // Round 18: 检查回归目的地是否被其他空闲机器人占据
                //   防止返回后挤占对方位置并触发新一轮避让循环
                if (!return_route_conflict) {
                  for (const auto & [oid, ost] : robot_statuses_) {
                    if (oid == robot_id) continue;
                    if (ost.current_waypoint == return_wp) {
                      return_route_conflict = true;
                      PersistLogger::log_info("avoidance.return_dest_occupied", robot_id, "",
                        "skip return to " + return_wp + ": " + oid +
                        " is at that waypoint",
                        __FILE__, __LINE__, __func__);
                      break;
                    }
                  }
                }
                if (!return_route_conflict) {
                  // Also guard against blocking corridors that are currently represented
                  // only in hold contexts (e.g. blocker route was just stale-recovered).
                  for (const auto & [hid, hctx] : hold_contexts_) {
                    if (hctx.release_wait_waypoint == return_wp) {
                      return_route_conflict = true;
                      PersistLogger::log_info("avoidance.return_hold_wait_conflict", robot_id, "",
                        "skip return to " + return_wp + ": hold " + hid +
                        " is waiting for that waypoint",
                        __FILE__, __LINE__, __func__);
                      break;
                    }
                    if (std::find(hctx.resume_route.begin(), hctx.resume_route.end(), return_wp) !=
                        hctx.resume_route.end())
                    {
                      return_route_conflict = true;
                      PersistLogger::log_info("avoidance.return_hold_resume_conflict", robot_id, "",
                        "skip return to " + return_wp + ": hold " + hid +
                        " resume route contains that waypoint",
                        __FILE__, __LINE__, __func__);
                      break;
                    }
                  }
                }
                if (!return_route_conflict) {
                PersistLogger::log_info("avoidance.return", robot_id, "",
                  "returning from " + cur_wp + " to " + return_wp +
                  " path_len=" + std::to_string(return_path.size()),
                  __FILE__, __LINE__, __func__);
                ni->route_waypoints = return_path;
                ni->current_waypoint_index = 0;
                ni->current_task_id = "avoidance_return_" + robot_id;
                navigate_to_next_waypoint(robot_id);
                return;
                }
              }
            }
          }
        }
      }
      // Round 11b: 清空路线数据，防止 route_conflict 检查到残留路线
      // 但如果正在等底盘反馈（chassis_task_sent），不能清空 current_task_id，
      // 否则 chassis_feedback_callback 会因 chassis_task_sent 被超时检查清零而忽略反馈
      if (!ni->chassis_task_sent) {
        ni->route_waypoints.clear();
        ni->current_waypoint_index = 0;
        ni->current_task_id.clear();
      }
    };

  nav_info->has_active_goal = true;
  nav_info->last_goal_issue_time = this->now();
  nav_info->nav_control_commit_until =
    nav_info->last_goal_issue_time + rclcpp::Duration::from_seconds(kNavControlCommitSec);
  nav_info->nav_active_since = nav_info->last_goal_issue_time;
  nav_info->nav_last_activity = nav_info->nav_active_since;
  nav_info->nav_client->async_send_goal(goal_msg, send_goal_options);
}

void FleetManagerNode::send_navigation_through_waypoints(
  const std::string & robot_id,
  const std::vector<std::string> & waypoint_path)
{
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info) return;
  nav_info->route_waypoints = waypoint_path;
  nav_info->current_waypoint_index = 0;
  nav_info->first_nav_pre_rotate_done = false;
  nav_info->first_nav_pre_rotate_task_id.clear();
  navigate_to_next_waypoint(robot_id);
}

}  // namespace fleet_manager
