#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#include <tf2/LinearMath/Quaternion.hpp>

namespace fleet_manager
{
// ==================== 工具 ====================

bool FleetManagerNode::robot_holds_waypoint_for_active_task(
  const std::string & robot_id, const std::string & waypoint_id) const
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto st_it = robot_statuses_.find(robot_id);
  if (st_it == robot_statuses_.end()) return false;
  if (st_it->second.location_type != "waypoint" || st_it->second.current_waypoint != waypoint_id) return false;
  for (const auto & task : task_scheduler_->get_all_tasks()) {
    if (task.assigned_robot_id != robot_id) continue;
    if (task.waypoint_id != waypoint_id) continue;
    if (task.status == "in_progress" || task.status == "assigned" || task.status == "waiting_fleet") return true;
  }
  return false;
}

bool FleetManagerNode::robot_yieldable_as_idle_blocker(const std::string & robot_id)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  // 离线机器人不可让行（无法移动它，尝试让行会永久阻塞请求者）
  {
    auto st_it = robot_statuses_.find(robot_id);
    if (st_it == robot_statuses_.end() || st_it->second.connection_status != "online") {
      return false;
    }
  }

  // Bug-fix: hard-held 且有用户任务的机器人通常不可让行，否则避让会覆盖其任务状态。
  // 但“交通清障等待型”的 hold（如 idle_blocking_wait_clear）本质上就是为了让行/解阻塞，
  // 若我们在这里一刀切 veto，会形成日志中的闭环：A 首跳被 B 锁住 -> B hard-hold 等 A 清路。
  if (is_robot_hard_held(robot_id)) {
    auto h_it = hold_contexts_.find(robot_id);
    if (h_it != hold_contexts_.end() &&
        !h_it->second.resume_task_id.empty() &&
        !is_internal_task_id(h_it->second.resume_task_id))
    {
      const auto & hctx = h_it->second;
      const bool traffic_clear_hold =
        (hctx.reason == "idle_blocking_wait_clear") ||
        (hctx.reason.rfind("idle_blocking", 0) == 0);
      if (!traffic_clear_hold) {
        return false;
      }
    }
  }

  // 正在等底盘执行反馈的机器人不可让行（执行器工作中，不能移动）
  if (is_robot_executing(robot_id)) {
    return false;
  }

  // 规则调整：
  // - 正在执行/导航中的用户任务不可让行；
  // - 仅 assigned/pending/waiting_fleet 且尚未真正起动导航的机器人，允许被让行调度，
  //   否则会出现“大家都 assigned 但都未起动、互相首跳锁死”的全局停滞。
  bool has_prestart_user_task = false;
  for (const auto & t : task_scheduler_->get_all_tasks()) {
    if (t.assigned_robot_id != robot_id) {
      continue;
    }
    if (t.status == "in_progress" || t.status == "executing") {
      return false;
    }
    if (t.status == "pending" || t.status == "assigned" || t.status == "waiting_fleet") {
      has_prestart_user_task = true;
    }
  }

  auto nav = get_robot_nav_info(robot_id);
  if (!nav) return true;

  if (has_prestart_user_task) {
    const bool nav_busy =
      nav->has_active_goal || nav->pre_rotate_pending ||
      !nav->pending_waypoint_path.empty() ||
      !nav->pending_through_segment_after_rotate.empty() ||
      nav->through_segment_mode;
    if (nav_busy) {
      return false;
    }
    // 用户任务只是“预起动”状态，允许临时让行。
    return true;
  }

  // 完全空闲（无任务）→ 可让行
  if (nav->current_task_id.empty()) return true;

  // Round 12d: 内部任务（避让/回归）正在导航中 → 不可让行
  if (is_internal_task_id(nav->current_task_id)) {
    if (nav->has_active_goal || nav->pre_rotate_pending || nav->through_segment_mode)
      return false;
    return true;  // 内部任务已完成，机器人闲置 → 可让行
  }

  // 用户任务：正在导航中（含 through_segment_mode）→ 不可让行
  return !(nav->has_active_goal || nav->pre_rotate_pending ||
           !nav->pending_waypoint_path.empty() ||
           !nav->pending_through_segment_after_rotate.empty() ||
           nav->through_segment_mode);
}

double FleetManagerNode::normalize_angle_rad(double a) const
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double FleetManagerNode::get_yaw_from_quat(const geometry_msgs::msg::Quaternion & q) const
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool FleetManagerNode::vlog_enabled() const
{
  if (!this->has_parameter("scheduler_verbose_log")) return scheduler_verbose_log_;
  try { return this->get_parameter("scheduler_verbose_log").as_bool(); }
  catch (...) { return scheduler_verbose_log_; }
}

std::string FleetManagerNode::nav_state_string(
  const std::string & robot_id,
  const std::shared_ptr<RobotNavigationInfo> & nav_info) const
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  if (!nav_info) return "robot=" + robot_id + " nav_info=null";
  return "robot=" + robot_id +
         " hold=" + std::string(is_robot_hard_held(robot_id) ? "1" : "0") +
         " pre_rotate=" + std::string(nav_info->pre_rotate_pending ? "1" : "0") +
         " active_goal=" + std::string(nav_info->has_active_goal ? "1" : "0") +
         " task=" + (nav_info->current_task_id.empty() ? "-" : nav_info->current_task_id) +
         " route=" + join_waypoints(nav_info->route_waypoints);
}

// Nav2 backwards-compat callbacks
void FleetManagerNode::goal_response_callback(
  const std::string & robot_id,
  std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto gh = future.get();
  auto nav_info = get_robot_nav_info(robot_id);
  if (!gh) {
    if (nav_info) nav_info->has_active_goal = false;
  }
}

void FleetManagerNode::feedback_callback(
  const std::string &, GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback>) {}

void FleetManagerNode::result_callback(
  const std::string & robot_id,
  const GoalHandleNavigateToPose::WrappedResult & result)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info) return;
  nav_info->has_active_goal = false;
  const std::string tid = nav_info->current_task_id;
  if (tid.empty()) return;
  const bool internal = is_internal_task_id(tid);
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    if (!internal) on_navigation_succeeded(robot_id, tid);
  } else {
    if (!internal) { task_scheduler_->fail_task(tid, "Navigation failed"); finalize_task_completion(robot_id, tid); }
  }
}

void FleetManagerNode::check_waypoint_arrival()
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  for (auto & [robot_id, nav_info] : robot_nav_info_) {
    if (!nav_info->has_active_goal || nav_info->current_task_id.empty()) continue;
    auto robot_it = robot_statuses_.find(robot_id);
    if (robot_it == robot_statuses_.end()) continue;
    auto task = task_scheduler_->get_task_info(nav_info->current_task_id);
    if (task.task_id.empty()) continue;
    auto wp_pose = traffic_manager_->get_waypoint_pose(task.waypoint_id);
    double dist = std::sqrt(
      std::pow(robot_it->second.current_pose.position.x - wp_pose.position.x, 2) +
      std::pow(robot_it->second.current_pose.position.y - wp_pose.position.y, 2));
    if (dist < waypoint_acceptance_radius_) {
      RCLCPP_INFO(this->get_logger(),
                  "event=nav.arrival_check task=%s robot=%s state_prev=in_progress state_new=in_progress reason=WAYPOINT_REACHED detail=waypoint:%s dist_m:%.2f",
                  nav_info->current_task_id.c_str(), robot_id.c_str(), task.waypoint_id.c_str(), dist);
    }
  }
}

}  // namespace fleet_manager
