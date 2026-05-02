#ifndef FLEET_MANAGER__OCCUPANCY_MANAGER_HPP_
#define FLEET_MANAGER__OCCUPANCY_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <functional>

namespace fleet_manager
{

enum class LocationType { WAYPOINT, SEGMENT, UNKNOWN };

struct DiscreteLocation
{
  LocationType type{LocationType::UNKNOWN};
  std::string waypoint_id;
  std::string segment_from;
  std::string segment_to;
  double distance{0.0};

  std::string segment_str() const
  {
    if (type != LocationType::SEGMENT) return "";
    return segment_from + "->" + segment_to;
  }
};

class OccupancyManager
{
public:
  using AdjacencyMap = std::map<std::string, std::vector<std::string>>;
  using PoseQuery   = std::function<geometry_msgs::msg::Pose(const std::string &)>;
  using RadiusQuery = std::function<double(const std::string &)>;

  explicit OccupancyManager(rclcpp::Node * node);

  void set_topology(const AdjacencyMap & adj, PoseQuery pq, RadiusQuery rq);

  // ── Location ──────────────────────────────────────────────
  DiscreteLocation update_location(
    const std::string & robot_id,
    const geometry_msgs::msg::Pose & pose,
    double capture_radius,
    double segment_lateral_max);

  void force_set_location(const std::string & robot_id, const DiscreteLocation & loc);
  void clear_robot(const std::string & robot_id);

  // ── Safety checks ─────────────────────────────────────────
  std::string can_enter(
    const std::string & robot_id,
    const std::string & from_wp,
    const std::string & to_wp) const;

  std::string waypoint_blocker(
    const std::string & robot_id,
    const std::string & wp_id) const;

  // ── Reservations ──────────────────────────────────────────
  bool reserve_next(
    const std::string & robot_id,
    const std::string & from_wp,
    const std::string & to_wp);

  void release_reservations(const std::string & robot_id);
  void release_locks(const std::string & robot_id);
  void expire_stale_reservations();

  // ── Queries ───────────────────────────────────────────────
  DiscreteLocation get_location(const std::string & robot_id) const;
  std::string get_zone_holder(const std::string & wp_id) const;
  bool is_zone_free_for(const std::string & robot_id, const std::string & wp_id) const;

  std::string find_nearest_free_waypoint(
    const std::string & from_wp,
    const std::vector<std::string> & exclude = {}) const;

  std::set<std::string> get_occupied_zones() const;
  std::map<std::string, DiscreteLocation> get_all_locations() const;

private:
  static std::string edge_key(const std::string & a, const std::string & b);
  double point_to_segment_distance(
    double px, double py, double x1, double y1, double x2, double y2) const;

  rclcpp::Node * node_;

  AdjacencyMap adjacency_;
  PoseQuery     pose_query_;
  RadiusQuery   radius_query_;
  std::set<std::string> all_waypoints_;
  std::set<std::pair<std::string, std::string>> all_edges_;

  // zone_locks_[W] = robot physically occupying zone(W)
  // Zone(W) = {W} ∪ all edges incident to W
  std::map<std::string, std::string> zone_locks_;

  // reservations_[R] = waypoint that R intends to enter next
  std::map<std::string, std::string> reservations_;
  std::map<std::string, rclcpp::Time> reservation_times_;

  std::map<std::string, DiscreteLocation> robot_locations_;

  static constexpr double kReservationTTL = 300.0;
};

}  // namespace fleet_manager

#endif  // FLEET_MANAGER__OCCUPANCY_MANAGER_HPP_
