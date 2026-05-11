#include "fleet_manager/traffic_manager.hpp"
#include <rclcpp/logging.hpp>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <queue>
#include <set>

namespace fleet_manager
{

// ============================================================================
// 路径规划 — 寻路
// ============================================================================

std::vector<std::string> TrafficManager::find_path(
  const std::string & from_waypoint,
  const std::string & to_waypoint)
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<std::string> path;

  // 构建邻接表
  std::map<std::string, std::vector<std::string>> adjacency;
  for (const auto & wp : current_map_.waypoints) {
    for (const auto & conn : wp.connections) {
      adjacency[wp.waypoint_id].push_back(conn);
    }
  }

  // BFS 搜索最短路径
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

  // 回溯构建路径
  std::string current = to_waypoint;
  while (!current.empty()) {
    path.push_back(current);
    current = parent[current];
  }
  std::reverse(path.begin(), path.end());

  RCLCPP_DEBUG(node_->get_logger(), "Path found: %s",
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

  // Dijkstra 加权最短路径：被占用的边/航点附加惩罚代价
  // 基础代价为航点间的欧氏距离
  // 被占用边的惩罚值 15.0，表示机器人宁愿绕行最多 15 米也不使用被占用的边
  static constexpr double kOccupiedEdgeCost = 15.0;
  static constexpr double kOccupiedWaypointCost = 10.0;

  // 构建邻接表与航点位置映射
  std::map<std::string, std::vector<std::string>> adjacency;
  std::map<std::string, geometry_msgs::msg::Pose> waypoint_poses;
  for (const auto & wp : current_map_.waypoints) {
    waypoint_poses[wp.waypoint_id] = wp.pose;
    for (const auto & conn : wp.connections) {
      adjacency[wp.waypoint_id].push_back(conn);
    }
  }

  // 边键归一化：确保 "A|B" 与 "B|A" 被视为同一条边
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

  // 小顶堆：(累积代价, 航点ID)
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

    if (cur_cost > dist[cur]) continue;  // 跳过过期条目

    for (const auto & neighbor : adjacency[cur]) {
      double edge_cost = calculate_distance(waypoint_poses[cur], waypoint_poses[neighbor]);
      // 被占用边附加惩罚
      std::string ek = make_edge_key(cur, neighbor);
      if (normalized_occupied_edges.count(ek)) {
        edge_cost += kOccupiedEdgeCost;
      }
      // 被占用航点附加惩罚（目标航点除外，必须确保可达）
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
    // 未找到路径时返回空，由调用方回退到普通 BFS
    return path;
  }

  // 回溯构建加权最短路径
  std::string current = to_waypoint;
  while (!current.empty()) {
    path.push_back(current);
    current = parent[current];
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<std::string> TrafficManager::find_path_avoiding(
  const std::string & from_waypoint,
  const std::string & to_waypoint,
  const std::set<std::string> & avoid_waypoints)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 边界:起点本身在 avoid_waypoints 中 → 仍允许从起点开始
  // 终点本身在 avoid_waypoints 中 → 允许进入终点
  // 中间航点在 avoid_waypoints 中 → 跳过
  std::map<std::string, std::vector<std::string>> adjacency;
  for (const auto & wp : current_map_.waypoints) {
    for (const auto & conn : wp.connections) {
      adjacency[wp.waypoint_id].push_back(conn);
    }
  }

  std::map<std::string, std::string> parent;
  std::set<std::string> visited;
  std::queue<std::string> q;
  q.push(from_waypoint);
  visited.insert(from_waypoint);
  parent[from_waypoint] = "";
  bool found = (from_waypoint == to_waypoint);

  while (!q.empty() && !found) {
    auto cur = q.front();
    q.pop();
    for (const auto & nb : adjacency[cur]) {
      if (visited.count(nb)) continue;
      // 硬避让:除终点外,avoid 的航点完全跳过
      if (nb != to_waypoint && avoid_waypoints.count(nb)) continue;
      visited.insert(nb);
      parent[nb] = cur;
      if (nb == to_waypoint) { found = true; break; }
      q.push(nb);
    }
  }

  std::vector<std::string> path;
  if (!found) return path;
  std::string cur = to_waypoint;
  while (!cur.empty()) {
    path.push_back(cur);
    cur = parent[cur];
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

  // 查找起点航点
  auto from_it = std::find_if(
    current_map_.waypoints.begin(), current_map_.waypoints.end(),
    [&from_waypoint](const fleet_msgs::msg::Waypoint & wp) {
      return wp.waypoint_id == from_waypoint;
    });

  // 查找终点航点
  auto to_it = std::find_if(
    current_map_.waypoints.begin(), current_map_.waypoints.end(),
    [&to_waypoint](const fleet_msgs::msg::Waypoint & wp) {
      return wp.waypoint_id == to_waypoint;
    });

  if (from_it == current_map_.waypoints.end() || to_it == current_map_.waypoints.end()) {
    RCLCPP_ERROR(node_->get_logger(), "Waypoint not found in map");
    return path;
  }

  // 插值生成路径点序列
  path = interpolate_path(from_it->pose, to_it->pose);
  return path;
}

// ============================================================================
// 航点查询
// ============================================================================

geometry_msgs::msg::Pose TrafficManager::get_waypoint_pose(
  const std::string & waypoint_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return get_waypoint_pose_unlocked(waypoint_id);
}

fleet_msgs::msg::TrafficMap TrafficManager::get_map() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_map_;
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

  // 遍历所有航点，找到欧氏距离最近的航点
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

// ============================================================================
// 拓扑暴露
// ============================================================================

std::map<std::string, std::vector<std::string>> TrafficManager::get_adjacency_map() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 构建并返回完整的邻接表
  std::map<std::string, std::vector<std::string>> adj;
  for (const auto & wp : current_map_.waypoints) {
    adj[wp.waypoint_id] = std::vector<std::string>(
      wp.connections.begin(), wp.connections.end());
  }
  return adj;
}

// ============================================================================
// 航点编辑
// ============================================================================

bool TrafficManager::add_waypoint(const fleet_msgs::msg::Waypoint & waypoint)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 检查航点是否已存在
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

// ============================================================================
// 栅格地图
// ============================================================================

void TrafficManager::set_occupancy_grid(
  const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  std::lock_guard<std::mutex> lock(mutex_);
  occupancy_grid_ = map;
}

// ============================================================================
// 几何工具
// ============================================================================

double TrafficManager::calculate_distance(
  const geometry_msgs::msg::Pose & p1,
  const geometry_msgs::msg::Pose & p2) const
{
  double dx = p1.position.x - p2.position.x;
  double dy = p1.position.y - p2.position.y;
  return std::sqrt(dx * dx + dy * dy);
}


}  // namespace fleet_manager
