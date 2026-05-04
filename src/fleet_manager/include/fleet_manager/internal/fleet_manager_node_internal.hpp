#ifndef FLEET_MANAGER__FLEET_MANAGER_NODE_INTERNAL_HPP_
#define FLEET_MANAGER__FLEET_MANAGER_NODE_INTERNAL_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>

namespace fleet_manager
{

// ============================================================================
// 内联工具函数 — 跨编译单元共享的无状态小函数
// ============================================================================

/// 判断是否为内部协调任务的 task_id(避让/链撤退/自动驶离)
inline bool is_internal_task_id(const std::string & task_id)
{
  return task_id.rfind("avoidance_", 0) == 0;
}

/// 将航点 ID 列表拼接为可读字符串: "wp_001->wp_002->...->wp_N"
inline std::string join_waypoints(const std::vector<std::string> & wps, size_t max_items = 12)
{
  std::string out;
  const size_t n = wps.size();
  const size_t take = std::min(n, max_items);
  for (size_t i = 0; i < take; ++i) {
    out += wps[i];
    if (i + 1 < take) out += "->";
  }
  if (n > take) out += "->...(" + std::to_string(n) + ")";
  return out;
}

/// 去除路径中连续重复的航点 ID
inline std::vector<std::string> dedup_consecutive_waypoints(const std::vector<std::string> & in)
{
  std::vector<std::string> out;
  out.reserve(in.size());
  for (const auto & id : in) {
    if (out.empty() || out.back() != id) out.push_back(id);
  }
  return out;
}

/// 等待 Nav2 action server 就绪(默认超时 120ms)
template<typename ActionT>
inline bool action_client_ready(
  const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(120))
{
  if (!client) return false;
  if (client->action_server_is_ready()) return true;
  return client->wait_for_action_server(timeout);
}

/// 角度归一化到 [-π, π]
inline double normalize_angle_rad(double a)
{
  while (a >  M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

/// 导航取消后的静默期(防止目标取消回调和新的导航指令冲突)
constexpr double kNavCancelSettlingSec = 0.6;

}  // namespace fleet_manager

#endif  // FLEET_MANAGER__FLEET_MANAGER_NODE_INTERNAL_HPP_
