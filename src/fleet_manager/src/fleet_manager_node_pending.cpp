#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#include <tf2/LinearMath/Quaternion.hpp>

namespace fleet_manager
{
// ==================== Pending 路径下发 ====================

void FleetManagerNode::pending_path_timer_callback()
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  const auto now = this->now();

  for (auto & [robot_id, nav_info] : robot_nav_info_) {
    if (!nav_info) continue;

    // hold 期间禁止通过 pending timer 重投导航，防止被 hold 的车被重新唤醒挪动。
    if (is_robot_hard_held(robot_id)) continue;

    // pre_rotate_pending 时不下发（等待 NavigateToPose result 回调处理）
    if (nav_info->pre_rotate_pending) continue;

    // 有活跃 goal 时不重复下发
    if (nav_info->has_active_goal) continue;

    // 等待 recent_cancel_until 保护期结束
    if (nav_info->recent_cancel_until.nanoseconds() > 0 && now < nav_info->recent_cancel_until) continue;

    // 单点导航模式不再使用 through-segment pending。
    if (!nav_info->pending_through_segment_after_rotate.empty()) {
      nav_info->pending_through_segment_after_rotate.clear();
    }

    // 下发 pending_waypoint_path
    if (!nav_info->pending_waypoint_path.empty()) {
      const bool ready =
        action_client_ready<NavigateToPose>(nav_info->nav_client, std::chrono::milliseconds(40));
      if (ready) {
        auto path = nav_info->pending_waypoint_path;
        nav_info->pending_waypoint_path.clear();
        nav_info->route_waypoints = path;
        nav_info->current_waypoint_index = 0;
        navigate_to_next_waypoint(robot_id);
      }
    }
  }
}

}  // namespace fleet_manager
