#include "fleet_manager/occupancy_manager.hpp"
#include "fleet_manager/persist_logger.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace fleet_manager
{

OccupancyManager::OccupancyManager(rclcpp::Node * node) : node_(node) {}

// ==================== Topology ====================

void OccupancyManager::set_topology(
  const AdjacencyMap & adj, PoseQuery pq, RadiusQuery rq)
{
  adjacency_   = adj;
  pose_query_  = std::move(pq);
  radius_query_ = std::move(rq);

  all_waypoints_.clear();
  all_edges_.clear();
  for (const auto & [wp, conns] : adjacency_) {
    all_waypoints_.insert(wp);
    for (const auto & c : conns) {
      all_waypoints_.insert(c);
      std::string a = std::min(wp, c);
      std::string b = std::max(wp, c);
      all_edges_.insert({a, b});
    }
  }

  for (auto it = zone_locks_.begin(); it != zone_locks_.end(); ) {
    if (all_waypoints_.find(it->first) == all_waypoints_.end())
      it = zone_locks_.erase(it);
    else ++it;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[OccupancyManager] topology: %zu waypoints, %zu edges",
    all_waypoints_.size(), all_edges_.size());
}

// ==================== Location ====================

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

  // waypoint first
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

  // segment fallback
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

  // release old zone locks held by this robot
  for (auto it = zone_locks_.begin(); it != zone_locks_.end(); ) {
    if (it->second == robot_id) it = zone_locks_.erase(it);
    else ++it;
  }

  // set new zone locks — never overwrite another robot's lock
  auto safe_set = [&](const std::string & wp) {
    auto existing = zone_locks_.find(wp);
    if (existing == zone_locks_.end() || existing->second == robot_id) {
      zone_locks_[wp] = robot_id;
    } else {
      PersistLogger::log_error(
        "occ.zone_collision", robot_id, "",
        "cannot claim wp=" + wp + " already held by " + existing->second +
        " (this robot location: " +
        (loc.type == LocationType::WAYPOINT ? "at " + loc.waypoint_id : "on " + loc.segment_str()) + ")",
        __FILE__, __LINE__, __func__);
    }
  };

  if (loc.type == LocationType::WAYPOINT) {
    safe_set(loc.waypoint_id);
  } else if (loc.type == LocationType::SEGMENT) {
    safe_set(loc.segment_from);
    safe_set(loc.segment_to);
  }

  robot_locations_[robot_id] = loc;

  // clear reservation if robot reached the reserved waypoint
  auto res_it = reservations_.find(robot_id);
  if (res_it != reservations_.end()) {
    if ((loc.type == LocationType::WAYPOINT && loc.waypoint_id == res_it->second) ||
        (loc.type == LocationType::SEGMENT &&
         (loc.segment_from == res_it->second || loc.segment_to == res_it->second))) {
      reservations_.erase(res_it);
      reservation_times_.erase(robot_id);
    }
  }

  return loc;
}

void OccupancyManager::force_set_location(
  const std::string & robot_id, const DiscreteLocation & loc)
{
  clear_robot(robot_id);
  robot_locations_[robot_id] = loc;
  if (loc.type == LocationType::WAYPOINT)
    zone_locks_[loc.waypoint_id] = robot_id;
  else if (loc.type == LocationType::SEGMENT) {
    zone_locks_[loc.segment_from] = robot_id;
    zone_locks_[loc.segment_to]   = robot_id;
  }
}

void OccupancyManager::clear_robot(const std::string & robot_id)
{
  robot_locations_.erase(robot_id);
  reservations_.erase(robot_id);
  reservation_times_.erase(robot_id);
  for (auto it = zone_locks_.begin(); it != zone_locks_.end(); ) {
    if (it->second == robot_id) it = zone_locks_.erase(it);
    else ++it;
  }
}

// ==================== Safety checks ====================

std::string OccupancyManager::can_enter(
  const std::string & robot_id,
  const std::string & from_wp,
  const std::string & to_wp) const
{
  if (to_wp.empty() || from_wp.empty() || to_wp == from_wp) return "invalid";

  // check zone lock (physical occupancy)
  auto zl = zone_locks_.find(to_wp);
  if (zl != zone_locks_.end() && zl->second != robot_id) return zl->second;

  // check reservations by other robots
  for (const auto & [rid, wp] : reservations_) {
    if (rid == robot_id) continue;
    if (wp == to_wp) return rid;
  }

  // check if edge from->to is occupied: robot on this edge holds both endpoint zones
  // already covered by zone_locks check above (both from and to would be locked)

  return "";  // clear
}

std::string OccupancyManager::waypoint_blocker(
  const std::string & robot_id,
  const std::string & wp_id) const
{
  if (wp_id.empty()) return "";
  auto zl = zone_locks_.find(wp_id);
  if (zl != zone_locks_.end() && zl->second != robot_id) return zl->second;
  for (const auto & [rid, wp] : reservations_) {
    if (rid == robot_id) continue;
    if (wp == wp_id) return rid;
  }
  return "";
}

// ==================== Reservations ====================

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

  reservations_[robot_id]     = to_wp;
  reservation_times_[robot_id] = node_->now();
  return true;
}

void OccupancyManager::release_reservations(const std::string & robot_id)
{
  reservations_.erase(robot_id);
  reservation_times_.erase(robot_id);
}

void OccupancyManager::release_locks(const std::string & robot_id)
{
  for (auto it = zone_locks_.begin(); it != zone_locks_.end(); ) {
    if (it->second == robot_id) it = zone_locks_.erase(it);
    else ++it;
  }
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
}

// ==================== Queries ====================

DiscreteLocation OccupancyManager::get_location(const std::string & robot_id) const
{
  auto it = robot_locations_.find(robot_id);
  return (it != robot_locations_.end()) ? it->second : DiscreteLocation{};
}

std::string OccupancyManager::get_zone_holder(const std::string & wp_id) const
{
  auto it = zone_locks_.find(wp_id);
  return (it != zone_locks_.end()) ? it->second : "";
}

bool OccupancyManager::is_zone_free_for(
  const std::string & robot_id, const std::string & wp_id) const
{
  auto zl = zone_locks_.find(wp_id);
  if (zl != zone_locks_.end() && zl->second != robot_id) return false;
  for (const auto & [rid, wp] : reservations_) {
    if (rid == robot_id) continue;
    if (wp == wp_id) return false;
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

  // neighbours first
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

  // global
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
  for (const auto & [_, wp] : reservations_) s.insert(wp);
  return s;
}

std::map<std::string, DiscreteLocation> OccupancyManager::get_all_locations() const
{
  return robot_locations_;
}

// ==================== Helpers ====================

std::string OccupancyManager::edge_key(const std::string & a, const std::string & b)
{
  return (a <= b) ? (a + "<->" + b) : (b + "<->" + a);
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

}  // namespace fleet_manager
