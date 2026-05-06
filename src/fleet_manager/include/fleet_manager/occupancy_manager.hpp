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

// ============================================================================
// 离散位置 — 将连续姿态映射到交通图的离散元素
// ============================================================================

enum class LocationType { WAYPOINT, SEGMENT, UNKNOWN };
enum class ResourceOwnerState { FREE, RESERVED, OCCUPIED, GHOST, CONFLICT };
enum class RobotResourceState { UNKNOWN, IDLE, RESERVED, MOVING, WAITING, EXECUTING, GHOST, CONFLICT };

struct DiscreteLocation
{
  LocationType type{LocationType::UNKNOWN};
  std::string waypoint_id;      // 所在航点(type==WAYPOINT)
  std::string segment_from;     // 所在航段起点(type==SEGMENT)
  std::string segment_to;       // 所在航段终点(type==SEGMENT)
  double distance{0.0};         // 到最近航点/航段的距离

  std::string segment_str() const
  {
    if (type != LocationType::SEGMENT) return "";
    return segment_from + "->" + segment_to;
  }
};

// ============================================================================
// OccupancyManager — 基于 Zone 的区域占用管理
//
// 核心概念:
//   Zone(W) = {航点W} ∪ {所有与W相邻的边}
//   一个底盘占据航点W时，它拥有整个 Zone(W)，包括 W 本身和所有连接边。
//   另一底盘想进入W(或其连接边)时必须等待当前占用者离开。
//
// 双重保护:
//   zone_locks_    — 物理占用: 底盘当前所在位置(由 update_location 维护)
//   reservations_  — 预留: 底盘即将前往的下一航点(由 reserve_next 创建)
//
// 幽灵锁:
//   底盘掉线后，zone_locks 保留作为"幽灵守卫"防止碰撞。
//   超时后自动 expire，防止永久封堵。
// ============================================================================

class OccupancyManager
{
public:
  using AdjacencyMap = std::map<std::string, std::vector<std::string>>;
  using PoseQuery   = std::function<geometry_msgs::msg::Pose(const std::string &)>;
  using RadiusQuery = std::function<double(const std::string &)>;

  explicit OccupancyManager(rclcpp::Node * node);

  /// 注入交通图拓扑和查询函数(地图加载/切换时调用)
  void set_topology(const AdjacencyMap & adj, PoseQuery pq, RadiusQuery rq);

  // ── 位置更新 ──────────────────────────────────────────────

  /// 根据底盘实时姿态更新离散位置并重新分配 zone_locks
  DiscreteLocation update_location(
    const std::string & robot_id,
    const geometry_msgs::msg::Pose & pose,
    double capture_radius,
    double segment_lateral_max);

  void force_set_location(const std::string & robot_id, const DiscreteLocation & loc);

  /// 完全清除底盘的所有占用状态(包括锁、预留、位置)
  void clear_robot(const std::string & robot_id);

  // ── 安全检查 ──────────────────────────────────────────────

  /// 检查底盘能否进入 to_wp: 返回空串=可进入, 返回robot_id=被谁阻塞
  std::string can_enter(
    const std::string & robot_id,
    const std::string & from_wp,
    const std::string & to_wp) const;

  /// 检查航点被谁阻塞(zone_lock 持有者或预留者)
  std::string waypoint_blocker(
    const std::string & robot_id,
    const std::string & wp_id) const;

  // ── 预留管理 ──────────────────────────────────────────────

  /// 为底盘预留下一个航点(会先做 can_enter 检查)
  bool reserve_next(
    const std::string & robot_id,
    const std::string & from_wp,
    const std::string & to_wp);

  void release_reservations(const std::string & robot_id);
  void release_locks(const std::string & robot_id);
  void expire_stale_reservations();       // 超时 TTL=300s

  // ── 幽灵锁(离线底盘的位置保护) ─────────────────────────

  /// 标记底盘的 zone_locks 为幽灵锁(记录时间戳)
  void mark_ghost(const std::string & robot_id, rclcpp::Time now);

  /// 清除幽灵标记(底盘重新上线时)
  void clear_ghost(const std::string & robot_id);

  /// 清理过期的幽灵锁(释放 zone_locks 和 reservations)
  void expire_ghost_locks(rclcpp::Time now, double ttl_sec);

  // ── 查询 ──────────────────────────────────────────────────

  DiscreteLocation get_location(const std::string & robot_id) const;

  /// 获取航点 zone 的当前持有者
  std::string get_zone_holder(const std::string & wp_id) const;

  /// 航点 zone 是否对该底盘可用
  bool is_zone_free_for(const std::string & robot_id, const std::string & wp_id) const;

  /// 从 from_wp 出发找到最近的空闲航点(用于避让目标计算)
  std::string find_nearest_free_waypoint(
    const std::string & from_wp,
    const std::vector<std::string> & exclude = {}) const;

  /// 获取所有被占用(zone_lock + reservation)的航点集合
  std::set<std::string> get_occupied_zones() const;

  std::map<std::string, DiscreteLocation> get_all_locations() const;
  std::map<std::string, std::set<std::string>> get_conflict_hubs() const;
  std::map<std::string, std::set<std::string>> get_conflict_edges() const;
  RobotResourceState get_robot_resource_state(const std::string & robot_id) const;

private:
  static std::string edge_key(const std::string & a, const std::string & b);
  static std::string conflict_key(const std::string & hub, const std::set<std::string> & holders);
  static std::string first_other_holder(const std::set<std::string> & holders, const std::string & robot_id);
  static std::string resource_state_name(ResourceOwnerState state);
  static std::string robot_state_name(RobotResourceState state);

  std::set<std::string> hubs_for_location(const DiscreteLocation & loc) const;
  void rebuild_resource_state();
  void set_robot_state(const std::string & robot_id, RobotResourceState state);

  double point_to_segment_distance(
    double px, double py, double x1, double y1, double x2, double y2) const;

  /// 检查持有 zone 的底盘是否为已过期的幽灵
  bool is_holder_active(const std::string & robot_id, rclcpp::Time now, double ttl_sec) const;

  rclcpp::Node * node_;

  AdjacencyMap adjacency_;
  PoseQuery     pose_query_;
  RadiusQuery   radius_query_;
  std::set<std::string> all_waypoints_;
  std::set<std::pair<std::string, std::string>> all_edges_;

  std::map<std::string, std::string> zone_locks_;           // 航点 → 占用底盘
  std::map<std::string, std::set<std::string>> reservations_; // 底盘 → 预留航点集合
  std::map<std::string, rclcpp::Time> reservation_times_;   // 底盘 → 预留时间

  std::map<std::string, DiscreteLocation> robot_locations_; // 底盘 → 离散位置
  std::map<std::string, std::set<std::string>> physical_hubs_;
  std::map<std::string, std::set<std::string>> physical_edges_;
  std::map<std::string, std::set<std::string>> ghost_hubs_;
  std::map<std::string, std::set<std::string>> conflict_hubs_;
  std::map<std::string, std::set<std::string>> conflict_edges_;
  std::map<std::string, std::set<std::string>> edge_reservations_;
  std::map<std::string, RobotResourceState> robot_states_;
  std::set<std::string> active_conflict_keys_;

  std::map<std::string, rclcpp::Time> ghost_locks_;         // 底盘 → 幽灵锁开始时间

  static constexpr double kReservationTTL = 300.0;           // 预留有效期
};

}  // namespace fleet_manager

#endif  // FLEET_MANAGER__OCCUPANCY_MANAGER_HPP_
