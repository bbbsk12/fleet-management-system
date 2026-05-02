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

inline bool is_internal_task_id(const std::string & task_id)
{
  return task_id.rfind("avoidance_", 0) == 0;
}

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

inline std::vector<std::string> dedup_consecutive_waypoints(const std::vector<std::string> & in)
{
  std::vector<std::string> out;
  out.reserve(in.size());
  for (const auto & id : in) {
    if (out.empty() || out.back() != id) out.push_back(id);
  }
  return out;
}

template<typename ActionT>
inline bool action_client_ready(
  const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(120))
{
  if (!client) return false;
  if (client->action_server_is_ready()) return true;
  return client->wait_for_action_server(timeout);
}

inline double normalize_angle_rad(double a)
{
  while (a >  M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

constexpr double kNavCancelSettlingSec = 0.6;

}  // namespace fleet_manager

#endif  // FLEET_MANAGER__FLEET_MANAGER_NODE_INTERNAL_HPP_
