#ifndef FLEET_MANAGER__TRAFFIC_MANAGER_HPP_
#define FLEET_MANAGER__TRAFFIC_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <fleet_msgs/msg/traffic_map.hpp>
#include <fleet_msgs/msg/waypoint.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <map>
#include <mutex>

namespace fleet_manager
{

// ============================================================================
// TrafficManager — 交通图管理、BFS 寻路、航点查询
//
// 职责边界: 仅负责地图数据的加载/保存、拓扑寻路和航点位姿查询。
// 所有占用/预留/冲突检测逻辑已迁移到 OccupancyManager。
// ============================================================================

class TrafficManager
{
public:
  explicit TrafficManager(rclcpp::Node * node);

  // ── 地图持久化 ────────────────────────────────────────────

  /// 从 YAML 文件加载交通图(航点 + 连接关系 + 栅格地图元信息)
  bool load_map(const std::string & file_path);

  /// 将当前交通图保存到 YAML 文件
  bool save_map(const std::string & file_path);

  /// 校验航点最小间距(min_spacing 应 > waypoint_acceptance_radius × 2)
  std::vector<std::pair<std::string, std::string>> validate_waypoint_spacing(
    double min_spacing) const;

  // ── 寻路 ──────────────────────────────────────────────────

  /// BFS 最短路径(不考虑占用)
  std::vector<std::string> find_path(
    const std::string & from_waypoint,
    const std::string & to_waypoint);

  /// 占用感知寻路: occupied 的航点/航段 cost 大幅提高, 优先选空闲路径
  std::vector<std::string> find_path_weighted(
    const std::string & from_waypoint,
    const std::string & to_waypoint,
    const std::set<std::string> & occupied_edges,
    const std::set<std::string> & occupied_waypoints);

  /// 硬避让寻路: avoid_waypoints 中的航点视为图中不存在,完全绕开
  /// (与 find_path_weighted 的软成本不同,这里是强约束;若没绕路返回空)
  std::vector<std::string> find_path_avoiding(
    const std::string & from_waypoint,
    const std::string & to_waypoint,
    const std::set<std::string> & avoid_waypoints);

  /// 两点间线性插值路径(用于可视化)
  std::vector<geometry_msgs::msg::Pose> plan_route(
    const std::string & from_waypoint,
    const std::string & to_waypoint);

  // ── 航点查询 ──────────────────────────────────────────────

  fleet_msgs::msg::TrafficMap get_map() const;

  geometry_msgs::msg::Pose get_waypoint_pose(const std::string & waypoint_id) const;
  std::map<std::string, geometry_msgs::msg::Pose> get_all_waypoint_poses() const;
  std::vector<std::string> get_waypoint_connections(const std::string & waypoint_id) const;
  double get_waypoint_radius(const std::string & waypoint_id) const;

  /// 在 max_distance 内查找距离位姿最近的航点
  std::string find_nearest_waypoint(
    const geometry_msgs::msg::Pose & pose,
    double max_distance = 2.0) const;

  /// 导出邻接表供 OccupancyManager 使用
  std::map<std::string, std::vector<std::string>> get_adjacency_map() const;

  // ── 航点编辑 ──────────────────────────────────────────────

  bool add_waypoint(const fleet_msgs::msg::Waypoint & waypoint);
  bool remove_waypoint(const std::string & waypoint_id);

  // ── 栅格地图 ──────────────────────────────────────────────

  void set_occupancy_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

  // ── 几何工具 ──────────────────────────────────────────────

  double calculate_distance(
    const geometry_msgs::msg::Pose & p1,
    const geometry_msgs::msg::Pose & p2) const;

private:
  geometry_msgs::msg::Pose get_waypoint_pose_unlocked(
    const std::string & waypoint_id) const;

  std::vector<geometry_msgs::msg::Pose> interpolate_path(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & end,
    double step_size = 0.5);

  bool check_collision(const geometry_msgs::msg::Pose & pose);
  void load_from_yaml(const YAML::Node & yaml);
  YAML::Node save_to_yaml();
  geometry_msgs::msg::Pose pixel_to_world(double pixel_x, double pixel_y) const;

  rclcpp::Node * node_;
  fleet_msgs::msg::TrafficMap current_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;

  double map_resolution_{0.05};
  double map_origin_x_{0.0};
  double map_origin_y_{0.0};
  int map_height_{0};
  std::string map_image_;
  std::string current_map_dir_;

  mutable std::mutex mutex_;
};

}  // namespace fleet_manager

#endif  // FLEET_MANAGER__TRAFFIC_MANAGER_HPP_
