#include "fleet_manager/traffic_manager.hpp"
#include <rclcpp/logging.hpp>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <queue>
#include <set>

namespace fleet_manager
{
// ==================== 寻路 ====================

std::vector<std::string> TrafficManager::find_path(
  const std::string & from_waypoint,
  const std::string & to_waypoint)
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<std::string> path;

  // BFS
  std::map<std::string, std::vector<std::string>> adjacency;
  for (const auto & wp : current_map_.waypoints) {
    for (const auto & conn : wp.connections) {
      adjacency[wp.waypoint_id].push_back(conn);
    }
  }

  std::queue<std::string> queue;
  std::map<std::string, std::string> parent;
  std::set<std::string> visited;

  queue.push(from_waypoint);
  visited.insert(from_waypoint);
  parent[from_waypoint] = "";

  bool found = false;

  while (!queue.empty() && !found) {
    std::string current = queue.front();
    queue.pop();

    if (current == to_waypoint) {
      found = true;
      break;
    }

    for (const auto & neighbor : adjacency[current]) {
      if (visited.find(neighbor) == visited.end()) {
        visited.insert(neighbor);
        parent[neighbor] = current;
        queue.push(neighbor);
      }
    }
  }

  if (!found) {
    RCLCPP_ERROR(node_->get_logger(), "No path found from %s to %s",
                 from_waypoint.c_str(), to_waypoint.c_str());
    return path;
  }

  std::string current = to_waypoint;
  while (!current.empty()) {
    path.push_back(current);
    current = parent[current];
  }
  std::reverse(path.begin(), path.end());

  RCLCPP_INFO(node_->get_logger(), "Path found: %s",
              [&path]() {
                std::string result;
                for (size_t i = 0; i < path.size(); ++i) {
                  result += path[i];
                  if (i < path.size() - 1) result += " -> ";
                }
                return result;
              }().c_str());

  return path;
}

std::vector<std::string> TrafficManager::find_path_weighted(
  const std::string & from_waypoint,
  const std::string & to_waypoint,
  const std::set<std::string> & occupied_edges,
  const std::set<std::string> & occupied_waypoints)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Dijkstra with weighted costs: occupied edges/waypoints get penalty
  // Base cost is the physical distance between waypoints.
  // P0 Fix: Reduced penalty to avoid massive detours. 
  // A penalty of 10.0 means the robot prefers a detour of up to 10 meters over an occupied edge.
  static constexpr double kOccupiedEdgeCost = 15.0;
  static constexpr double kOccupiedWaypointCost = 10.0;

  std::map<std::string, std::vector<std::string>> adjacency;
  std::map<std::string, geometry_msgs::msg::Pose> waypoint_poses;
  for (const auto & wp : current_map_.waypoints) {
    waypoint_poses[wp.waypoint_id] = wp.pose;
    for (const auto & conn : wp.connections) {
      adjacency[wp.waypoint_id].push_back(conn);
    }
  }

  auto make_edge_key = [](const std::string & a, const std::string & b) -> std::string {
    return (a < b) ? (a + "|" + b) : (b + "|" + a);
  };
  auto normalize_edge_key = [](const std::string & key) -> std::string {
    auto pos = key.find("<->");
    if (pos != std::string::npos) {
      const std::string a = key.substr(0, pos);
      const std::string b = key.substr(pos + 3);
      if (!a.empty() && !b.empty()) {
        return (a < b) ? (a + "|" + b) : (b + "|" + a);
      }
    }
    pos = key.find("|");
    if (pos != std::string::npos) {
      const std::string a = key.substr(0, pos);
      const std::string b = key.substr(pos + 1);
      if (!a.empty() && !b.empty()) {
        return (a < b) ? (a + "|" + b) : (b + "|" + a);
      }
    }
    return key;
  };
  std::set<std::string> normalized_occupied_edges;
  for (const auto & key : occupied_edges) {
    normalized_occupied_edges.insert(normalize_edge_key(key));
  }

  // min-heap: (cost, waypoint_id)
  using PQEntry = std::pair<double, std::string>;
  std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;
  std::map<std::string, double> dist;
  std::map<std::string, std::string> parent;

  dist[from_waypoint] = 0.0;
  parent[from_waypoint] = "";
  pq.push({0.0, from_waypoint});

  bool found = false;

  while (!pq.empty()) {
    auto [cur_cost, cur] = pq.top();
    pq.pop();

    if (cur == to_waypoint) {
      found = true;
      break;
    }

    if (cur_cost > dist[cur]) continue;  // stale entry

    for (const auto & neighbor : adjacency[cur]) {
      double edge_cost = calculate_distance(waypoint_poses[cur], waypoint_poses[neighbor]);
      std::string ek = make_edge_key(cur, neighbor);
      if (normalized_occupied_edges.count(ek)) {
        edge_cost += kOccupiedEdgeCost;
      }
      // 目标航点被占用时加惩罚（目的地除外，必须可达）
      if (neighbor != to_waypoint && occupied_waypoints.count(neighbor)) {
        edge_cost += kOccupiedWaypointCost;
      }

      double new_cost = cur_cost + edge_cost;
      auto dit = dist.find(neighbor);
      if (dit == dist.end() || new_cost < dit->second) {
        dist[neighbor] = new_cost;
        parent[neighbor] = cur;
        pq.push({new_cost, neighbor});
      }
    }
  }

  std::vector<std::string> path;
  if (!found) {
    // 回退到普通 BFS
    return path;
  }

  std::string current = to_waypoint;
  while (!current.empty()) {
    path.push_back(current);
    current = parent[current];
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<geometry_msgs::msg::Pose> TrafficManager::plan_route(
  const std::string & from_waypoint,
  const std::string & to_waypoint)
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<geometry_msgs::msg::Pose> path;

  auto from_it = std::find_if(
    current_map_.waypoints.begin(), current_map_.waypoints.end(),
    [&from_waypoint](const fleet_msgs::msg::Waypoint & wp) {
      return wp.waypoint_id == from_waypoint;
    });

  auto to_it = std::find_if(
    current_map_.waypoints.begin(), current_map_.waypoints.end(),
    [&to_waypoint](const fleet_msgs::msg::Waypoint & wp) {
      return wp.waypoint_id == to_waypoint;
    });

  if (from_it == current_map_.waypoints.end() || to_it == current_map_.waypoints.end()) {
    RCLCPP_ERROR(node_->get_logger(), "Waypoint not found in map");
    return path;
  }

  path = interpolate_path(from_it->pose, to_it->pose);
  return path;
}

// ==================== 航点查询 ====================

geometry_msgs::msg::Pose TrafficManager::get_waypoint_pose(
  const std::string & waypoint_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return get_waypoint_pose_unlocked(waypoint_id);
}

std::map<std::string, geometry_msgs::msg::Pose> TrafficManager::get_all_waypoint_poses() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::map<std::string, geometry_msgs::msg::Pose> poses;
  for (const auto & waypoint : current_map_.waypoints) {
    poses[waypoint.waypoint_id] = waypoint.pose;
  }
  return poses;
}

std::vector<std::string> TrafficManager::get_waypoint_connections(
  const std::string & waypoint_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = std::find_if(
    current_map_.waypoints.begin(), current_map_.waypoints.end(),
    [&waypoint_id](const fleet_msgs::msg::Waypoint & wp) {
      return wp.waypoint_id == waypoint_id;
    });

  if (it != current_map_.waypoints.end()) {
    return std::vector<std::string>(it->connections.begin(), it->connections.end());
  }
  return {};
}

double TrafficManager::get_waypoint_radius(const std::string & waypoint_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = std::find_if(
    current_map_.waypoints.begin(), current_map_.waypoints.end(),
    [&waypoint_id](const fleet_msgs::msg::Waypoint & wp) {
      return wp.waypoint_id == waypoint_id;
    });

  if (it != current_map_.waypoints.end()) {
    return static_cast<double>(it->radius);
  }
  return 0.5;
}

std::string TrafficManager::find_nearest_waypoint(
  const geometry_msgs::msg::Pose & pose,
  double max_distance) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::string nearest_wp;
  double min_dist = max_distance;

  for (const auto & wp : current_map_.waypoints) {
    double dist = std::sqrt(
      std::pow(pose.position.x - wp.pose.position.x, 2) +
      std::pow(pose.position.y - wp.pose.position.y, 2));
    if (dist < min_dist) {
      min_dist = dist;
      nearest_wp = wp.waypoint_id;
    }
  }
  return nearest_wp;
}

// ==================== 拓扑暴露 ====================

std::map<std::string, std::vector<std::string>> TrafficManager::get_adjacency_map() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::map<std::string, std::vector<std::string>> adj;
  for (const auto & wp : current_map_.waypoints) {
    adj[wp.waypoint_id] = std::vector<std::string>(
      wp.connections.begin(), wp.connections.end());
  }
  return adj;
}

// ==================== 航点编辑 ====================

bool TrafficManager::add_waypoint(const fleet_msgs::msg::Waypoint & waypoint)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = std::find_if(
    current_map_.waypoints.begin(), current_map_.waypoints.end(),
    [&waypoint](const fleet_msgs::msg::Waypoint & wp) {
      return wp.waypoint_id == waypoint.waypoint_id;
    });

  if (it != current_map_.waypoints.end()) {
    RCLCPP_WARN(node_->get_logger(), "Waypoint %s already exists", waypoint.waypoint_id.c_str());
    return false;
  }

  current_map_.waypoints.push_back(waypoint);
  current_map_.modified_at = node_->now();
  return true;
}

bool TrafficManager::remove_waypoint(const std::string & waypoint_id)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = std::remove_if(
    current_map_.waypoints.begin(), current_map_.waypoints.end(),
    [&waypoint_id](const fleet_msgs::msg::Waypoint & wp) {
      return wp.waypoint_id == waypoint_id;
    });

  if (it != current_map_.waypoints.end()) {
    current_map_.waypoints.erase(it, current_map_.waypoints.end());
    current_map_.modified_at = node_->now();
    return true;
  }
  return false;
}

// ==================== 栅格地图 ====================

void TrafficManager::set_occupancy_grid(
  const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  std::lock_guard<std::mutex> lock(mutex_);
  occupancy_grid_ = map;
}

// ==================== 几何工具 ====================

double TrafficManager::calculate_distance(
  const geometry_msgs::msg::Pose & p1,
  const geometry_msgs::msg::Pose & p2) const
{
  double dx = p1.position.x - p2.position.x;
  double dy = p1.position.y - p2.position.y;
  return std::sqrt(dx * dx + dy * dy);
}


}  // namespace fleet_manager
