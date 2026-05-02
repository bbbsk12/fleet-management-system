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

// ==================== 分段导航 ====================

std::vector<size_t> FleetManagerNode::compute_navigation_goal_indices(
  const std::vector<std::string> & full_route) const
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  std::vector<size_t> goal_indices;
  if (full_route.empty()) return goal_indices;

  const size_t n = full_route.size();
  goal_indices.push_back(0);

  for (size_t i = 1; i + 1 < n; ++i) {
    const auto p_prev = traffic_manager_->get_waypoint_pose(full_route[i - 1]);
    const auto p_cur = traffic_manager_->get_waypoint_pose(full_route[i]);
    const auto p_next = traffic_manager_->get_waypoint_pose(full_route[i + 1]);

    auto valid = [](const geometry_msgs::msg::Pose & p) {
      return !(std::abs(p.position.x) < 1e-6 && std::abs(p.position.y) < 1e-6);
    };
    if (!valid(p_prev) || !valid(p_cur) || !valid(p_next)) {
      goal_indices.push_back(i);
      continue;
    }

    const double v1x = p_cur.position.x - p_prev.position.x;
    const double v1y = p_cur.position.y - p_prev.position.y;
    const double v2x = p_next.position.x - p_cur.position.x;
    const double v2y = p_next.position.y - p_cur.position.y;
    const double n1 = std::hypot(v1x, v1y), n2 = std::hypot(v2x, v2y);
    if (n1 < 1e-6 || n2 < 1e-6) { goal_indices.push_back(i); continue; }

    double dot = std::clamp((v1x * v2x + v1y * v2y) / (n1 * n2), -1.0, 1.0);
    if (std::acos(dot) > route_turn_angle_threshold_rad_) {
      goal_indices.push_back(i);
    }
  }

  if (n > 1 && goal_indices.back() != n - 1) goal_indices.push_back(n - 1);
  return goal_indices;
}

std::vector<std::vector<std::string>> FleetManagerNode::split_route_into_turn_segments(
  const std::vector<std::string> & full_route) const
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  std::vector<std::vector<std::string>> segments;
  if (full_route.empty()) return segments;
  const auto g = compute_navigation_goal_indices(full_route);
  if (g.size() < 2) { segments.push_back(full_route); return segments; }
  for (size_t k = 0; k + 1 < g.size(); ++k) {
    std::vector<std::string> seg;
    for (size_t i = g[k]; i <= g[k + 1] && i < full_route.size(); ++i) {
      seg.push_back(full_route[i]);
    }
    if (!seg.empty()) segments.push_back(std::move(seg));
  }
  if (segments.empty()) segments.push_back(full_route);
  return segments;
}

double FleetManagerNode::bearing_pose_to_waypoint(
  const geometry_msgs::msg::Pose & pose, const std::string & waypoint_id) const
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  const auto wp = traffic_manager_->get_waypoint_pose(waypoint_id);
  return std::atan2(wp.position.y - pose.position.y, wp.position.x - pose.position.x);
}

void FleetManagerNode::reset_through_segment_state(const std::shared_ptr<RobotNavigationInfo> & nav_info)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  if (!nav_info) return;
  nav_info->through_segment_mode = false;
  nav_info->through_planner_full_route.clear();
  nav_info->through_route_segments.clear();
  nav_info->through_segment_index = 0;
  nav_info->pending_through_segment_after_rotate.clear();
  nav_info->through_advance_after_navigate_to_pose = false;
}

void FleetManagerNode::request_pre_rotate_then_through_segment(
  const std::string & robot_id,
  const std::vector<std::string> & segment_path)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info || segment_path.empty()) return;
  nav_info->first_nav_pre_rotate_done = true;
  nav_info->first_nav_pre_rotate_task_id = nav_info->current_task_id;

  auto st_it = robot_statuses_.find(robot_id);
  if (st_it == robot_statuses_.end()) {
    navigate_to_next_waypoint(robot_id);
    return;
  }

  const auto & pose = st_it->second.current_pose;

  // 检查位置数据有效性：如果位置全 0（无定位），跳过 pre_rotate 直接导航
  const bool pose_valid = !(std::abs(pose.position.x) < 1e-6 && std::abs(pose.position.y) < 1e-6);
  if (!pose_valid) {
    RCLCPP_INFO(this->get_logger(),
      "Robot %s has no valid pose (0,0), skipping pre_rotate for segment", robot_id.c_str());
    navigate_to_next_waypoint(robot_id);
    return;
  }

  // 计算目标朝向：从当前位置到下一个航路点
  const double desired_yaw = (segment_path.size() >= 2)
    ? bearing_pose_to_waypoint(pose, segment_path[1])
    : bearing_pose_to_waypoint(pose, segment_path[0]);

  // 计算转角误差
  const double current_yaw = get_yaw_from_quat(pose.orientation);
  const double turn_err = std::abs(normalize_angle_rad(desired_yaw - current_yaw));

  // 强制策略：首次导航前无条件执行对正，不再按阈值跳过。

  const std::string segment_signature = join_waypoints(segment_path);
  const auto now = this->now();

  // 转角超过阈值 → 用 NavigateToPose 原地转向
  // 构造 goal：位置保持当前，朝向设为目标方向
  cancel_active_robot_goals(nav_info);
  const uint64_t cmd_seq = ++nav_info->nav_command_seq;

  // 保存段信息，等 NavigateToPose result 回来后继续
  nav_info->pending_through_segment_after_rotate = segment_path;
  nav_info->pending_waypoint_path.clear();
  nav_info->pre_rotate_pending = true;
  // 预旋转本质也是一个活跃导航目标；必须刷新活动时间戳，
  // 否则 nav.stuck_recover 可能用旧时间戳误判并立刻回收。
  nav_info->nav_active_since = this->now();
  nav_info->nav_last_activity = nav_info->nav_active_since;

  geometry_msgs::msg::PoseStamped rotate_goal;
  rotate_goal.header.frame_id = "map";
  rotate_goal.header.stamp = this->now();
  rotate_goal.pose = pose;  // 位置保持不变
  tf2::Quaternion q;
  q.setRPY(0, 0, desired_yaw);
  rotate_goal.pose.orientation.x = q.x();
  rotate_goal.pose.orientation.y = q.y();
  rotate_goal.pose.orientation.z = q.z();
  rotate_goal.pose.orientation.w = q.w();

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = rotate_goal;

  // 如果 goal_tolerance 参数可用则设置（Nav2 默认使用 controller 的 tolerance）
  // 某些 Nav2 版本支持 goal_tolerance_xxx 参数，这里不做强依赖

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this, robot_id, cmd_seq](GoalHandleNavigateToPose::SharedPtr goal_handle) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      auto ni = get_robot_nav_info(robot_id);
      if (!ni || ni->nav_command_seq != cmd_seq) return;
      if (!goal_handle) {
        RCLCPP_WARN(this->get_logger(),
          "Pre-rotate NavigateToPose rejected for %s, falling back to direct navigation", robot_id.c_str());
        // Root-cause fix: keep nav state consistent on goal rejection.
        ni->has_active_goal = false;
        ni->current_pose_goal_handle.reset();
        ni->nav_active_since = rclcpp::Time{};
        ni->nav_last_activity = rclcpp::Time{};
        ni->pre_rotate_pending = false;
        ni->pending_through_segment_after_rotate.clear();
        navigate_to_next_waypoint(robot_id);
      } else {
        ni->current_pose_goal_handle = goal_handle;
        ni->nav_last_activity = this->now();
        // 预旋转也是“导航执行中”，需要对外发布 in_progress
        if (!ni->current_task_id.empty() && !is_internal_task_id(ni->current_task_id)) {
          task_scheduler_->mark_task_navigating(ni->current_task_id);
          auto latest = task_scheduler_->get_task_info(ni->current_task_id);
          if (!latest.task_id.empty()) {
            latest.status = "in_progress";
            task_status_pub_->publish(latest);
          }
        }
        PersistLogger::log_info(
          "nav.pre_rotate", robot_id, ni->current_task_id,
          "Pre-rotate NavigateToPose goal accepted",
          __FILE__, __LINE__, __func__);
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
      if (!ni || ni->nav_command_seq != cmd_seq) return;
      ni->has_active_goal = false;
      ni->current_pose_goal_handle.reset();
      ni->pre_rotate_pending = false;
      ni->nav_active_since = rclcpp::Time{};
      ni->nav_last_activity = rclcpp::Time{};

      auto seg = ni->pending_through_segment_after_rotate;
      ni->pending_through_segment_after_rotate.clear();

      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        PersistLogger::log_info(
          "nav.pre_rotate", robot_id, ni->current_task_id,
          "Pre-rotate completed, starting single-point navigation",
          __FILE__, __LINE__, __func__);
        (void)seg;
        navigate_to_next_waypoint(robot_id);
      } else {
        // 被取消（可能是 hold）或失败 → 仍继续导航流程，由上层状态机兜底
        PersistLogger::log_warn(
          "nav.pre_rotate", robot_id, ni->current_task_id,
          "Pre-rotate NavigateToPose result=" + std::to_string(static_cast<int>(result.code)) +
          ", falling back to direct navigation",
          __FILE__, __LINE__, __func__);
        (void)seg;
        navigate_to_next_waypoint(robot_id);
      }
    };

  // 检查 NavigateToPose action server 可用性
  const bool pose_ready = action_client_ready<NavigateToPose>(nav_info->nav_client);
  if (!nav_info->nav_client || !pose_ready) {
    RCLCPP_WARN(this->get_logger(),
      "NavigateToPose not ready for pre-rotate %s, skipping to direct navigation", robot_id.c_str());
    nav_info->pre_rotate_pending = false;
    nav_info->pending_through_segment_after_rotate.clear();
    navigate_to_next_waypoint(robot_id);
    return;
  }

  nav_info->has_active_goal = true;
  nav_info->last_goal_issue_time = this->now();
  nav_info->nav_control_commit_until =
    nav_info->last_goal_issue_time + rclcpp::Duration::from_seconds(kNavControlCommitSec);
  nav_info->last_pre_rotate_segment_signature = segment_signature;
  nav_info->last_pre_rotate_request_time = now;
  nav_info->nav_client->async_send_goal(goal_msg, send_goal_options);

  PersistLogger::log_info(
    "nav.pre_rotate", robot_id, nav_info->current_task_id,
    std::string("Pre-rotate requested: mode=") +
    (pre_rotate_every_segment_ ? "every_segment" : "threshold") +
    " turn=" + std::to_string(turn_err * 180.0 / M_PI) +
    "deg to segment " + join_waypoints(segment_path),
    __FILE__, __LINE__, __func__);
}

void FleetManagerNode::advance_through_segments_or_complete_task(
  const std::string & robot_id, bool success, const std::string & failure_reason)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info || !nav_info->through_segment_mode) return;

  const std::string task_id = nav_info->current_task_id;
  const bool internal_task = is_internal_task_id(task_id);
  const auto completed_segment_index = nav_info->through_segment_index;

  if (!success) {
    occupancy_manager_->release_reservations(robot_id);
    reset_through_segment_state(nav_info);
    nav_info->has_active_goal = false;
    nav_info->route_waypoints.clear();
    nav_info->current_waypoint_index = 0;
    if (!task_id.empty() && !internal_task) {
      task_scheduler_->fail_task(task_id, failure_reason.empty() ? "Through-segment failed" : failure_reason);
      auto latest = task_scheduler_->get_task_info(task_id);
      if (!latest.task_id.empty()) task_status_pub_->publish(latest);
      finalize_task_completion(robot_id, task_id);
    } else {
      nav_info->current_task_id.clear();
    }
    return;
  }

  const auto & full_route = nav_info->through_planner_full_route.empty()
    ? nav_info->route_waypoints
    : nav_info->through_planner_full_route;
  if (completed_segment_index < nav_info->through_route_segments.size() && !full_route.empty()) {
    const auto & completed_segment = nav_info->through_route_segments[completed_segment_index];
    if (!completed_segment.empty()) {
      const auto end_it = std::find(
        full_route.begin(), full_route.end(), completed_segment.back());
      if (end_it != full_route.end()) {
        nav_info->current_waypoint_index = static_cast<size_t>(
          std::distance(full_route.begin(), end_it));
      }
    }
  }

  nav_info->through_segment_index++;
  if (nav_info->through_segment_index < nav_info->through_route_segments.size()) {
    // Round 10: 段间推进前检查 reserve_next_hop，防止冲入其它机器人占用的航段
    const auto & next_seg = nav_info->through_route_segments[nav_info->through_segment_index];
    if (next_seg.size() >= 2) {
      const std::string & from_wp = next_seg[0];
      const std::string & to_wp   = next_seg[1];
      occupancy_manager_->release_reservations(robot_id);
      if (!occupancy_manager_->reserve_next_hop(robot_id, from_wp, to_wp)) {
        const std::string blocker = occupancy_manager_->check_can_enter(robot_id, from_wp, to_wp);
        PersistLogger::log_warn(
          "nav.through_seg_reserve", robot_id, task_id,
          "segment " + std::to_string(nav_info->through_segment_index) +
          " reserve denied from=" + from_wp + " to=" + to_wp + " blocker=" + blocker,
          __FILE__, __LINE__, __func__);
        if (!blocker.empty()) {
          apply_hard_hold(robot_id, "through_seg_wait_peer_clear", blocker, false, true);
          hold_contexts_[robot_id].release_wait_waypoint = to_wp;
          if (robot_yieldable_as_idle_blocker(blocker)) {
            std::vector<std::string> exclude(nav_info->route_waypoints.begin(), nav_info->route_waypoints.end());
            std::string avoid_wp = occupancy_manager_->find_avoidance_waypoint(to_wp, exclude);
            if (!avoid_wp.empty()) {
              request_move_idle_robot_to_avoidance(blocker, avoid_wp, /*force=*/false);
            }
          }
        }
        return;  // hold 释放后 navigate_to_next_waypoint 会以 sparse 模式恢复
      }
    }
    request_pre_rotate_then_through_segment(
      robot_id, nav_info->through_route_segments[nav_info->through_segment_index]);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Robot %s completed all through-segments for task %s",
              robot_id.c_str(), task_id.c_str());
  if (!task_id.empty() && !internal_task) {
    on_navigation_succeeded(robot_id, task_id);
    nav_abort_retry_count_.erase(task_id);
    // on_navigation_succeeded 可能设置 chassis_task_sent=true（LOAD/UNLOAD/SITE_SPECIFIC）
    // 此时不能清空 current_task_id，否则底盘反馈会被忽略
    if (!nav_info->chassis_task_sent) {
      nav_info->current_task_id.clear();
      nav_info->route_waypoints.clear();
      nav_info->current_waypoint_index = 0;
    }
  } else {
    // internal task or empty task_id: release immediately
    occupancy_manager_->release_reservations(robot_id);
    reset_through_segment_state(nav_info);
    nav_info->current_task_id.clear();
    nav_info->route_waypoints.clear();
    nav_info->current_waypoint_index = 0;
  }
}

void FleetManagerNode::on_through_navigation_action_finished(
  const std::string & robot_id, uint64_t cmd_seq,
  rclcpp_action::ResultCode code, bool /*used_follow_waypoints*/)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info || nav_info->nav_command_seq != cmd_seq || !nav_info->through_segment_mode) {
    if (nav_info) {
      PersistLogger::log_warn("nav.through_action_guard", robot_id, nav_info->current_task_id,
        "on_through_navigation_action_finished skipped: cmd_seq=" + std::to_string(cmd_seq) +
        " current_seq=" + std::to_string(nav_info->nav_command_seq) +
        " through_mode=" + std::to_string(nav_info->through_segment_mode) +
        " code=" + std::to_string(static_cast<int>(code)),
        __FILE__, __LINE__, __func__);
    }
    return;
  }

  nav_info->has_active_goal = false;
  nav_info->current_goal_handle.reset();
  nav_info->current_follow_goal_handle.reset();

  const std::string task_id = nav_info->current_task_id;
  const bool internal = is_internal_task_id(task_id);

  // Always log the terminal result for through-segment navigation.
  // Without this, a quick cancel/abort can look like "dispatched then silent".
  PersistLogger::log_info(
    "nav.result", robot_id, task_id,
    "ThroughNav result code=" + std::to_string(static_cast<int>(code)),
    __FILE__, __LINE__, __func__);

  if (code == rclcpp_action::ResultCode::SUCCEEDED) {
    advance_through_segments_or_complete_task(robot_id, true, "");
    return;
  }

  // Hold cancel 保护
  if (code == rclcpp_action::ResultCode::CANCELED) {
    if (hold_contexts_.find(robot_id) != hold_contexts_.end()) {
      PersistLogger::log_info("nav.result", robot_id, task_id,
        "ThroughNav CANCELED during hold, skipping fail", __FILE__, __LINE__, __func__);
      return;
    }
    // Cancellation often comes from our own state transitions (avoidance/hold/cancel settling).
    // For user tasks, prefer rescheduling instead of failing/clearing silently.
    if (!internal && !task_id.empty()) {
      reset_through_segment_state(nav_info);
      task_scheduler_->mark_task_pending(task_id);
      auto latest = task_scheduler_->get_task_info(task_id);
      if (!latest.task_id.empty()) task_status_pub_->publish(latest);
      return;
    }
  }

  if (code == rclcpp_action::ResultCode::ABORTED && !internal) {
    int & retries = nav_abort_retry_count_[task_id];
    if (retries < nav_abort_max_retries_) {
      ++retries;
      reset_through_segment_state(nav_info);
      task_scheduler_->mark_task_pending(task_id);
      return;
    }
    task_scheduler_->fail_task(task_id, "Navigation aborted");
    nav_abort_retry_count_.erase(task_id);
  } else if (!internal) {
    task_scheduler_->fail_task(task_id, "Navigation failed");
  }

  // 失败时释放锁和清理状态
  if (!internal) {
    finalize_task_completion(robot_id, task_id);
  } else {
    reset_through_segment_state(nav_info);
    nav_info->current_task_id.clear();
    nav_info->route_waypoints.clear();
    nav_info->current_waypoint_index = 0;
  }
}

}  // namespace fleet_manager
