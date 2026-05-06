#include "fleet_manager/occupancy_manager.hpp"
#include "fleet_manager/persist_logger.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace fleet_manager
{

OccupancyManager::OccupancyManager(rclcpp::Node * node) : node_(node) {}

// ============================================================================
// 拓扑注入
// ============================================================================

void OccupancyManager::set_topology(
  const AdjacencyMap & adj, PoseQuery pq, RadiusQuery rq)
{
  adjacency_    = adj;
  pose_query_   = std::move(pq);
  radius_query_ = std::move(rq);

  // 重建全部航点和边集合
  all_waypoints_.clear();
  all_edges_.clear();
  for (const auto & [wp, conns] : adjacency_) {
    all_waypoints_.insert(wp);
    for (const auto & c : conns) {
      all_waypoints_.insert(c);
      all_edges_.insert({std::min(wp, c), std::max(wp, c)});
    }
  }

  // 清除已不存在的航点的 zone_locks
  for (auto it = zone_locks_.begin(); it != zone_locks_.end(); ) {
    if (all_waypoints_.find(it->first) == all_waypoints_.end())
      it = zone_locks_.erase(it);
    else ++it;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[OccupancyManager] topology: %zu waypoints, %zu edges",
    all_waypoints_.size(), all_edges_.size());
  rebuild_resource_state();
}

// ============================================================================
// 位置更新 — 将连续姿态映射为离散航点/航段，并维护 zone_locks
// ============================================================================

DiscreteLocation OccupancyManager::update_location(
  const std::string & robot_id,
  const geometry_msgs::msg::Pose & pose,
  double capture_radius,
  double segment_lateral_max)
{
  DiscreteLocation loc;

  if (!pose_query_ || !radius_query_) {
    robot_locations_[robot_id] = loc;
    return loc;
  }

  // 第一步: 按 capture_radius 判定是否在航点范围内
  std::string best_wp;
  double best_wp_dist = std::numeric_limits<double>::max();
  for (const auto & wp_id : all_waypoints_) {
    const auto wp_pose = pose_query_(wp_id);
    const double dx = pose.position.x - wp_pose.position.x;
    const double dy = pose.position.y - wp_pose.position.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    const double cap = std::max(capture_radius, radius_query_(wp_id));
    if (dist <= cap && dist < best_wp_dist) {
      best_wp_dist = dist;
      best_wp = wp_id;
    }
  }

  if (!best_wp.empty()) {
    loc.type = LocationType::WAYPOINT;
    loc.waypoint_id = best_wp;
    loc.distance = best_wp_dist;
  }

  // 第二步: 不在任何航点 → 判定是否在航段上
  if (loc.type == LocationType::UNKNOWN) {
    double best_seg_dist = std::numeric_limits<double>::max();
    std::string best_from, best_to;
    for (const auto & [a, b] : all_edges_) {
      const auto pa = pose_query_(a);
      const auto pb = pose_query_(b);
      const double d = point_to_segment_distance(
        pose.position.x, pose.position.y,
        pa.position.x, pa.position.y,
        pb.position.x, pb.position.y);
      if (d < best_seg_dist) { best_seg_dist = d; best_from = a; best_to = b; }
    }
    const double lateral = std::max(segment_lateral_max, 0.8);
    if (!best_from.empty() && best_seg_dist <= lateral) {
      loc.type = LocationType::SEGMENT;
      loc.segment_from = best_from;
      loc.segment_to   = best_to;
      loc.distance     = best_seg_dist;
    }
  }

  robot_locations_[robot_id] = loc;

  // 到达预留航点时从集合中移除该航点
  auto res_it = reservations_.find(robot_id);
  if (res_it != reservations_.end()) {
    if (loc.type == LocationType::WAYPOINT)
      res_it->second.erase(loc.waypoint_id);
    else if (loc.type == LocationType::SEGMENT) {
      res_it->second.erase(loc.segment_from);
      res_it->second.erase(loc.segment_to);
    }
    if (res_it->second.empty()) {
      reservations_.erase(res_it);
      reservation_times_.erase(robot_id);
    }
  }

  rebuild_resource_state();

  return loc;
}

void OccupancyManager::force_set_location(
  const std::string & robot_id, const DiscreteLocation & loc)
{
  clear_robot(robot_id);
  robot_locations_[robot_id] = loc;
  rebuild_resource_state();
}

void OccupancyManager::clear_robot(const std::string & robot_id)
{
  robot_locations_.erase(robot_id);
  reservations_.erase(robot_id);
  reservation_times_.erase(robot_id);
  ghost_locks_.erase(robot_id);
  robot_states_.erase(robot_id);
  rebuild_resource_state();
}

// ============================================================================
// 安全检查
// ============================================================================

std::string OccupancyManager::can_enter(
  const std::string & robot_id,
  const std::string & from_wp,
  const std::string & to_wp) const
{
  if (to_wp.empty() || from_wp.empty() || to_wp == from_wp) return "invalid";
  if (!conflict_hubs_.empty()) {
    for (const auto & [hub, holders] : conflict_hubs_) {
      if (hub == from_wp || hub == to_wp) return first_other_holder(holders, robot_id);
    }
  }
  const std::string move_edge = edge_key(from_wp, to_wp);
  auto ce = conflict_edges_.find(move_edge);
  if (ce != conflict_edges_.end()) return first_other_holder(ce->second, robot_id);

  const std::set<std::string> footprint{from_wp, to_wp};
  for (const auto & wp : footprint) {
    auto zl = zone_locks_.find(wp);
    if (zl != zone_locks_.end() && zl->second != robot_id) return zl->second;
    for (const auto & [rid, wps] : reservations_) {
      if (rid == robot_id) continue;
      if (wps.count(wp)) return rid;
    }
  }
  for (const auto & [rid, edges] : edge_reservations_) {
    if (rid == robot_id) continue;
    if (edges.count(move_edge)) return rid;
  }

  return "";  // 畅通
}

std::string OccupancyManager::waypoint_blocker(
  const std::string & robot_id,
  const std::string & wp_id) const
{
  if (wp_id.empty()) return "";
  auto ch = conflict_hubs_.find(wp_id);
  if (ch != conflict_hubs_.end()) return first_other_holder(ch->second, robot_id);
  auto zl = zone_locks_.find(wp_id);
  if (zl != zone_locks_.end() && zl->second != robot_id) return zl->second;
  for (const auto & [rid, wps] : reservations_) {
    if (rid == robot_id) continue;
    if (wps.count(wp_id)) return rid;
  }
  return "";
}

// ============================================================================
// 预留管理
// ============================================================================

bool OccupancyManager::reserve_next(
  const std::string & robot_id,
  const std::string & from_wp,
  const std::string & to_wp)
{
  const std::string blocker = can_enter(robot_id, from_wp, to_wp);
  if (!blocker.empty()) {
    PersistLogger::log_warn(
      "occ.reserve_denied", robot_id, "",
      "from=" + from_wp + " to=" + to_wp + " blocker=" + blocker,
      __FILE__, __LINE__, __func__);
    return false;
  }

  reservations_[robot_id].insert(from_wp);
  reservations_[robot_id].insert(to_wp);
  reservation_times_[robot_id] = node_->now();
  set_robot_state(robot_id, RobotResourceState::RESERVED);
  rebuild_resource_state();
  return true;
}

void OccupancyManager::release_reservations(const std::string & robot_id)
{
  reservations_.erase(robot_id);
  reservation_times_.erase(robot_id);
  rebuild_resource_state();
}

void OccupancyManager::release_locks(const std::string & robot_id)
{
  robot_locations_.erase(robot_id);
  rebuild_resource_state();
}

void OccupancyManager::expire_stale_reservations()
{
  const auto now = node_->now();
  std::vector<std::string> expired;
  for (const auto & [rid, t] : reservation_times_) {
    if ((now - t).seconds() > kReservationTTL) expired.push_back(rid);
  }
  for (const auto & rid : expired) {
    PersistLogger::log_warn(
      "occ.reservation_expired", rid, "",
      "reservation TTL expired, releasing",
      __FILE__, __LINE__, __func__);
    reservations_.erase(rid);
    reservation_times_.erase(rid);
  }
  if (!expired.empty()) rebuild_resource_state();
}

// ============================================================================
// 查询
// ============================================================================

DiscreteLocation OccupancyManager::get_location(const std::string & robot_id) const
{
  auto it = robot_locations_.find(robot_id);
  return (it != robot_locations_.end()) ? it->second : DiscreteLocation{};
}

std::string OccupancyManager::get_zone_holder(const std::string & wp_id) const
{
  auto ch = conflict_hubs_.find(wp_id);
  if (ch != conflict_hubs_.end()) return first_other_holder(ch->second, "");
  auto it = zone_locks_.find(wp_id);
  return (it != zone_locks_.end()) ? it->second : "";
}

bool OccupancyManager::is_zone_free_for(
  const std::string & robot_id, const std::string & wp_id) const
{
  if (conflict_hubs_.count(wp_id)) return false;
  auto zl = zone_locks_.find(wp_id);
  if (zl != zone_locks_.end() && zl->second != robot_id) return false;
  for (const auto & [rid, wps] : reservations_) {
    if (rid == robot_id) continue;
    if (wps.count(wp_id)) return false;
  }
  return true;
}

std::string OccupancyManager::find_nearest_free_waypoint(
  const std::string & from_wp,
  const std::vector<std::string> & exclude) const
{
  if (!pose_query_) return "";

  const auto from_pose = pose_query_(from_wp);
  std::string best;
  double best_dist = std::numeric_limits<double>::max();

  // 优先搜邻居
  auto adj_it = adjacency_.find(from_wp);
  if (adj_it != adjacency_.end()) {
    for (const auto & nb : adj_it->second) {
      if (std::find(exclude.begin(), exclude.end(), nb) != exclude.end()) continue;
      if (!is_zone_free_for("", nb)) continue;
      const auto p = pose_query_(nb);
      const double d = std::hypot(p.position.x - from_pose.position.x,
                                   p.position.y - from_pose.position.y);
      if (d < best_dist && d > 0.1) { best_dist = d; best = nb; }
    }
  }

  if (!best.empty()) return best;

  // 全局搜索
  for (const auto & wp : all_waypoints_) {
    if (wp == from_wp) continue;
    if (std::find(exclude.begin(), exclude.end(), wp) != exclude.end()) continue;
    if (!is_zone_free_for("", wp)) continue;
    const auto p = pose_query_(wp);
    const double d = std::hypot(p.position.x - from_pose.position.x,
                                 p.position.y - from_pose.position.y);
    if (d < best_dist && d > 0.1) { best_dist = d; best = wp; }
  }

  return best;
}

std::set<std::string> OccupancyManager::get_occupied_zones() const
{
  std::set<std::string> s;
  for (const auto & [wp, _] : zone_locks_) s.insert(wp);
  for (const auto & [wp, _] : conflict_hubs_) s.insert(wp);
  for (const auto & [_, wps] : reservations_)
    for (const auto & wp : wps) s.insert(wp);
  return s;
}

std::map<std::string, DiscreteLocation> OccupancyManager::get_all_locations() const
{
  return robot_locations_;
}

std::map<std::string, std::set<std::string>> OccupancyManager::get_conflict_hubs() const
{
  return conflict_hubs_;
}

std::map<std::string, std::set<std::string>> OccupancyManager::get_conflict_edges() const
{
  return conflict_edges_;
}

RobotResourceState OccupancyManager::get_robot_resource_state(const std::string & robot_id) const
{
  auto it = robot_states_.find(robot_id);
  return (it != robot_states_.end()) ? it->second : RobotResourceState::UNKNOWN;
}

std::set<std::string> OccupancyManager::hubs_for_location(const DiscreteLocation & loc) const
{
  std::set<std::string> hubs;
  if (loc.type == LocationType::WAYPOINT && !loc.waypoint_id.empty()) {
    hubs.insert(loc.waypoint_id);
  } else if (loc.type == LocationType::SEGMENT) {
    if (!loc.segment_from.empty()) hubs.insert(loc.segment_from);
    if (!loc.segment_to.empty()) hubs.insert(loc.segment_to);
  }
  return hubs;
}

void OccupancyManager::rebuild_resource_state()
{
  zone_locks_.clear();
  physical_hubs_.clear();
  physical_edges_.clear();
  ghost_hubs_.clear();
  conflict_hubs_.clear();
  conflict_edges_.clear();
  edge_reservations_.clear();
  robot_states_.clear();

  for (const auto & [rid, loc] : robot_locations_) {
    auto hubs = hubs_for_location(loc);
    if (hubs.empty()) {
      set_robot_state(rid, RobotResourceState::IDLE);
      continue;
    }
    for (const auto & hub : hubs) physical_hubs_[hub].insert(rid);
    if (loc.type == LocationType::WAYPOINT) set_robot_state(rid, RobotResourceState::IDLE);
    else if (loc.type == LocationType::SEGMENT) {
      physical_edges_[edge_key(loc.segment_from, loc.segment_to)].insert(rid);
      set_robot_state(rid, RobotResourceState::MOVING);
    }
  }

  for (const auto & [rid, _] : ghost_locks_) {
    auto loc_it = robot_locations_.find(rid);
    if (loc_it == robot_locations_.end()) continue;
    auto hubs = hubs_for_location(loc_it->second);
    for (const auto & hub : hubs) ghost_hubs_[hub].insert(rid);
    set_robot_state(rid, RobotResourceState::GHOST);
  }

  for (const auto & [rid, wps] : reservations_) {
    if (!wps.empty() && !ghost_locks_.count(rid)) {
      if (wps.size() == 2) {
        auto it = wps.begin();
        const std::string a = *it++;
        const std::string b = *it;
        edge_reservations_[edge_key(a, b)].insert(rid);
      }
      auto st = get_robot_resource_state(rid);
      if (st == RobotResourceState::UNKNOWN || st == RobotResourceState::IDLE)
        set_robot_state(rid, RobotResourceState::RESERVED);
    }
  }

  std::set<std::string> next_conflicts;
  for (const auto & [hub, holders] : physical_hubs_) {
    if (holders.size() <= 1) {
      if (!holders.empty()) zone_locks_[hub] = *holders.begin();
      continue;
    }
    conflict_hubs_[hub] = holders;
    next_conflicts.insert(conflict_key(hub, holders));
    for (const auto & rid : holders) set_robot_state(rid, RobotResourceState::CONFLICT);
  }

  for (const auto & [edge, holders] : physical_edges_) {
    if (holders.size() <= 1) continue;
    conflict_edges_[edge] = holders;
    next_conflicts.insert(conflict_key(edge, holders));
    for (const auto & rid : holders) set_robot_state(rid, RobotResourceState::CONFLICT);
  }

  for (const auto & [hub, holders] : conflict_hubs_) {
    const auto key = conflict_key(hub, holders);
    if (!active_conflict_keys_.count(key)) {
      std::ostringstream oss;
      bool first = true;
      for (const auto & rid : holders) {
        if (!first) oss << ",";
        first = false;
        oss << rid;
      }
      PersistLogger::log_error("occ.hub_conflict", "",
        "",
        "hub=" + hub + " holders=" + oss.str(),
        __FILE__, __LINE__, __func__);
    }
  }
  for (const auto & [edge, holders] : conflict_edges_) {
    const auto key = conflict_key(edge, holders);
    if (!active_conflict_keys_.count(key)) {
      std::ostringstream oss;
      bool first = true;
      for (const auto & rid : holders) {
        if (!first) oss << ",";
        first = false;
        oss << rid;
      }
      PersistLogger::log_error("occ.edge_conflict", "",
        "",
        "edge=" + edge + " holders=" + oss.str(),
        __FILE__, __LINE__, __func__);
    }
  }
  active_conflict_keys_ = std::move(next_conflicts);
}

void OccupancyManager::set_robot_state(const std::string & robot_id, RobotResourceState state)
{
  if (robot_id.empty()) return;
  robot_states_[robot_id] = state;
}

// ============================================================================
// 几何辅助
// ============================================================================

std::string OccupancyManager::edge_key(const std::string & a, const std::string & b)
{
  return (a <= b) ? (a + "<->" + b) : (b + "<->" + a);
}

std::string OccupancyManager::conflict_key(const std::string & hub, const std::set<std::string> & holders)
{
  std::string key = hub + ":";
  for (const auto & rid : holders) key += rid + "|";
  return key;
}

std::string OccupancyManager::first_other_holder(
  const std::set<std::string> & holders, const std::string & robot_id)
{
  for (const auto & rid : holders) {
    if (rid != robot_id) return rid;
  }
  return holders.empty() ? "" : *holders.begin();
}

std::string OccupancyManager::resource_state_name(ResourceOwnerState state)
{
  switch (state) {
    case ResourceOwnerState::FREE: return "FREE";
    case ResourceOwnerState::RESERVED: return "RESERVED";
    case ResourceOwnerState::OCCUPIED: return "OCCUPIED";
    case ResourceOwnerState::GHOST: return "GHOST";
    case ResourceOwnerState::CONFLICT: return "CONFLICT";
  }
  return "UNKNOWN";
}

std::string OccupancyManager::robot_state_name(RobotResourceState state)
{
  switch (state) {
    case RobotResourceState::UNKNOWN: return "UNKNOWN";
    case RobotResourceState::IDLE: return "IDLE";
    case RobotResourceState::RESERVED: return "RESERVED";
    case RobotResourceState::MOVING: return "MOVING";
    case RobotResourceState::WAITING: return "WAITING";
    case RobotResourceState::EXECUTING: return "EXECUTING";
    case RobotResourceState::GHOST: return "GHOST";
    case RobotResourceState::CONFLICT: return "CONFLICT";
  }
  return "UNKNOWN";
}

double OccupancyManager::point_to_segment_distance(
  double px, double py, double x1, double y1, double x2, double y2) const
{
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  if (dx == 0 && dy == 0) return std::hypot(px - x1, py - y1);
  double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(px - (x1 + t * dx), py - (y1 + t * dy));
}

// ============================================================================
// 幽灵锁 — 离线底盘的延迟清理
// ============================================================================

void OccupancyManager::mark_ghost(const std::string & robot_id, rclcpp::Time now)
{
  ghost_locks_[robot_id] = now;
  rebuild_resource_state();
  PersistLogger::log_info(
    "occ.ghost", robot_id, "",
    "zone locks marked as ghost (will expire after TTL)",
    __FILE__, __LINE__, __func__);
}

void OccupancyManager::clear_ghost(const std::string & robot_id)
{
  ghost_locks_.erase(robot_id);
  rebuild_resource_state();
}

bool OccupancyManager::is_holder_active(const std::string & robot_id, rclcpp::Time now, double ttl_sec) const
{
  auto it = ghost_locks_.find(robot_id);
  if (it == ghost_locks_.end()) return true;  // 非幽灵 → 活跃
  double age = (now - it->second).seconds();
  return age < ttl_sec;  // TTL 内 → 仍视为活跃
}

void OccupancyManager::expire_ghost_locks(rclcpp::Time now, double ttl_sec)
{
  std::vector<std::string> expired;
  for (const auto & [rid, ghost_time] : ghost_locks_) {
    if ((now - ghost_time).seconds() >= ttl_sec) expired.push_back(rid);
  }
  for (const auto & rid : expired) {
    PersistLogger::log_warn(
      "occ.ghost_expired", rid, "",
      "ghost lock TTL expired, releasing all zone locks",
      __FILE__, __LINE__, __func__);
    release_locks(rid);
    release_reservations(rid);
    ghost_locks_.erase(rid);
    rebuild_resource_state();
  }
}

}  // namespace fleet_manager
