#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#include <tf2/LinearMath/Quaternion.hpp>

namespace fleet_manager
{
// ==================== LED 灯带控制 ====================

uint8_t FleetManagerNode::determine_led_state(const std::string & robot_id) const
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto nav_it = robot_nav_info_.find(robot_id);
  if (nav_it == robot_nav_info_.end() || !nav_it->second) {
    return fleet_msgs::msg::LEDTask::STATE_IDLE;
  }
  const auto & nav_info = nav_it->second;

  // 优先级1：底盘正在执行任务（LOAD/UNLOAD/SITE_SPECIFIC）
  if (nav_info->chassis_task_sent) {
    return fleet_msgs::msg::LEDTask::STATE_TASK_EXECUTING;
  }

  // 优先级2：交通等待（被 hold、避让中等）
  if (is_robot_hard_held(robot_id)) {
    return fleet_msgs::msg::LEDTask::STATE_TRAFFIC_WAIT;
  }

  // 优先级3：正在导航（有活跃 goal、pre_rotate、有任务 ID）
  const bool navigating =
    nav_info->has_active_goal ||
    nav_info->pre_rotate_pending ||
    !nav_info->current_task_id.empty() ||
    !nav_info->route_waypoints.empty() ||
    !nav_info->pending_waypoint_path.empty() ||
    nav_info->through_segment_mode;

  if (navigating) {
    return fleet_msgs::msg::LEDTask::STATE_WALKING;
  }

  // 优先级4：空闲
  return fleet_msgs::msg::LEDTask::STATE_IDLE;
}

void FleetManagerNode::led_timer_callback()
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  for (auto & [robot_id, nav_info] : robot_nav_info_) {
    if (!nav_info) continue;
    if (!nav_info->led_task_pub) continue;

    // 检查机器人是否在线
    auto st_it = robot_statuses_.find(robot_id);
    if (st_it == robot_statuses_.end() || st_it->second.connection_status != "online") {
      continue;
    }

    const uint8_t state = determine_led_state(robot_id);

    fleet_msgs::msg::LEDTask msg;
    msg.state = state;
    nav_info->led_task_pub->publish(msg);
    nav_info->last_led_state = state;
  }
}

void FleetManagerNode::led_status_callback(
  const std::string & robot_id,
  const fleet_msgs::msg::LEDStatus::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto nav_it = robot_nav_info_.find(robot_id);
  if (nav_it == robot_nav_info_.end() || !nav_it->second) return;

  nav_it->second->chassis_led_state = msg->state;
  nav_it->second->chassis_led_received = msg->received;
}

}  // namespace fleet_manager
