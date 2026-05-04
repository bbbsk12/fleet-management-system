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
// 地图加载/保存
// ============================================================================

bool TrafficManager::load_map(const std::string & file_path)
{
  std::lock_guard<std::mutex> lock(mutex_);

  try {
    YAML::Node yaml = YAML::LoadFile(file_path);

    // 提取地图文件所在目录（用于解析相对路径）
    size_t last_slash = file_path.find_last_of("/\\");
    if (last_slash != std::string::npos) {
      current_map_dir_ = file_path.substr(0, last_slash);
    } else {
      current_map_dir_ = ".";
    }

    load_from_yaml(yaml);

    current_map_.modified_at = node_->now();
    RCLCPP_INFO(node_->get_logger(), "Loaded traffic map from %s", file_path.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load map: %s", e.what());
    return false;
  }
}

std::vector<std::pair<std::string, std::string>> TrafficManager::validate_waypoint_spacing(
  double min_spacing) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::pair<std::string, std::string>> violations;

  // 检查所有航点对之间的距离是否满足最小间距要求
  const auto & wps = current_map_.waypoints;
  for (size_t i = 0; i < wps.size(); ++i) {
    for (size_t j = i + 1; j < wps.size(); ++j) {
      const double dx = wps[i].pose.position.x - wps[j].pose.position.x;
      const double dy = wps[i].pose.position.y - wps[j].pose.position.y;
      const double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_spacing) {
        violations.push_back({wps[i].waypoint_id, wps[j].waypoint_id});
        RCLCPP_WARN(node_->get_logger(),
          "Waypoint spacing violation: %s and %s are %.3fm apart (min=%.3fm)",
          wps[i].waypoint_id.c_str(), wps[j].waypoint_id.c_str(), dist, min_spacing);
      }
    }
  }

  if (!violations.empty()) {
    RCLCPP_WARN(node_->get_logger(),
      "Map validation: %zu waypoint pairs have spacing < %.3fm. "
      "This may cause location detection jitter and lock oscillation.",
      violations.size(), min_spacing);
  }

  return violations;
}

bool TrafficManager::save_map(const std::string & file_path)
{
  std::lock_guard<std::mutex> lock(mutex_);

  try {
    YAML::Node yaml = save_to_yaml();
    std::ofstream fout(file_path);
    fout << yaml;

    RCLCPP_INFO(node_->get_logger(), "Saved traffic map to %s", file_path.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to save map: %s", e.what());
    return false;
  }
}

// ============================================================================
// 内部工具 — Private
// ============================================================================

geometry_msgs::msg::Pose TrafficManager::get_waypoint_pose_unlocked(
  const std::string & waypoint_id) const
{
  auto it = std::find_if(
    current_map_.waypoints.begin(), current_map_.waypoints.end(),
    [&waypoint_id](const fleet_msgs::msg::Waypoint & wp) {
      return wp.waypoint_id == waypoint_id;
    });

  if (it != current_map_.waypoints.end()) {
    return it->pose;
  }

  geometry_msgs::msg::Pose empty_pose;
  return empty_pose;
}

std::vector<geometry_msgs::msg::Pose> TrafficManager::interpolate_path(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & end,
  double step_size)
{
  std::vector<geometry_msgs::msg::Pose> path;
  path.push_back(start);

  // 计算起点到终点的欧氏距离，按步长等分插值
  double distance = std::sqrt(
    std::pow(end.position.x - start.position.x, 2) +
    std::pow(end.position.y - start.position.y, 2));

  int steps = static_cast<int>(distance / step_size);

  for (int i = 1; i < steps; ++i) {
    double t = static_cast<double>(i) / steps;
    geometry_msgs::msg::Pose pose;
    pose.position.x = start.position.x + t * (end.position.x - start.position.x);
    pose.position.y = start.position.y + t * (end.position.y - start.position.y);
    pose.position.z = start.position.z + t * (end.position.z - start.position.z);
    pose.orientation = start.orientation;
    path.push_back(pose);
  }

  path.push_back(end);
  return path;
}

bool TrafficManager::check_collision(const geometry_msgs::msg::Pose & pose)
{
  if (!occupancy_grid_) {
    return false;
  }

  // 将世界坐标转为栅格坐标
  int x = static_cast<int>((pose.position.x - occupancy_grid_->info.origin.position.x) /
                           occupancy_grid_->info.resolution);
  int y = static_cast<int>((pose.position.y - occupancy_grid_->info.origin.position.y) /
                           occupancy_grid_->info.resolution);

  if (x >= 0 && x < static_cast<int>(occupancy_grid_->info.width) &&
      y >= 0 && y < static_cast<int>(occupancy_grid_->info.height))
  {
    int index = y * occupancy_grid_->info.width + x;
    return occupancy_grid_->data[index] > 50;
  }

  return true;
}

geometry_msgs::msg::Pose TrafficManager::pixel_to_world(double pixel_x, double pixel_y) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = pixel_x * map_resolution_ + map_origin_x_;
  pose.position.y = pixel_y * map_resolution_ + map_origin_y_;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

void TrafficManager::load_from_yaml(const YAML::Node & yaml)
{
  current_map_.waypoints.clear();
  map_height_ = 0;
  map_image_.clear();

  // --- 地图元数据 ---
  if (yaml["map_resolution"]) {
    map_resolution_ = yaml["map_resolution"].as<double>();
  }
  if (yaml["map_origin"] && yaml["map_origin"].IsSequence() && yaml["map_origin"].size() >= 2) {
    map_origin_x_ = yaml["map_origin"][0].as<double>();
    map_origin_y_ = yaml["map_origin"][1].as<double>();
  }

  RCLCPP_INFO(node_->get_logger(), "Map metadata: resolution=%.3f, origin=(%.2f, %.2f), dir=%s",
              map_resolution_, map_origin_x_, map_origin_y_, current_map_dir_.c_str());

  // --- 从 map_image 加载图像高度 ---
  if (yaml["map_image"]) {
    std::string map_image = yaml["map_image"].as<std::string>();
    map_image_ = map_image;

    std::string full_image_path;
    if (!map_image.empty() && map_image[0] != '/') {
      full_image_path = current_map_dir_ + "/" + map_image;
    } else {
      full_image_path = map_image;
    }

    RCLCPP_INFO(node_->get_logger(), "Trying to load map image from: %s", full_image_path.c_str());

    std::ifstream pgm_file(full_image_path, std::ios::binary);
    if (pgm_file.is_open()) {
      std::string magic, line;
      int width, height, maxval;
      pgm_file >> magic;
      RCLCPP_INFO(node_->get_logger(), "PGM magic: %s", magic.c_str());
      if (magic == "P5" || magic == "P2") {
        while (pgm_file.peek() == '#') {
          std::getline(pgm_file, line);
        }
        pgm_file >> width >> height >> maxval;
        map_height_ = height;
        RCLCPP_INFO(node_->get_logger(),
                    "Loaded map image height from map_image: %d (width: %d, path: %s)",
                    map_height_, width, full_image_path.c_str());
      } else {
        RCLCPP_WARN(node_->get_logger(), "Invalid PGM magic: %s", magic.c_str());
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "Could not open map image: %s (errno: %d)",
                  full_image_path.c_str(), errno);
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "No map_image field in yaml");
  }

  // --- 若 map_image 加载失败，尝试从 map_yaml_path 加载图像高度 ---
  if (map_height_ == 0 && yaml["map_yaml_path"]) {
    std::string map_yaml_path = yaml["map_yaml_path"].as<std::string>();

    std::string full_yaml_path;
    if (!map_yaml_path.empty() && map_yaml_path[0] != '/') {
      full_yaml_path = current_map_dir_ + "/" + map_yaml_path;
    } else {
      full_yaml_path = map_yaml_path;
    }

    try {
      YAML::Node map_yaml = YAML::LoadFile(full_yaml_path);
      if (map_yaml["image"]) {
        std::string image_path = map_yaml["image"].as<std::string>();

        std::string full_image_path;
        if (!image_path.empty() && image_path[0] != '/') {
          full_image_path = current_map_dir_ + "/" + image_path;
        } else {
          full_image_path = image_path;
        }

        std::ifstream pgm_file(full_image_path, std::ios::binary);
        if (pgm_file.is_open()) {
          std::string magic, line;
          int width, height, maxval;
          pgm_file >> magic;
          if (magic == "P5" || magic == "P2") {
            while (pgm_file.peek() == '#') {
              std::getline(pgm_file, line);
            }
            pgm_file >> width >> height >> maxval;
            map_height_ = height;
            RCLCPP_INFO(node_->get_logger(),
                        "Loaded map image height from map_yaml_path: %d", map_height_);
          }
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(), "Could not load map image height from %s: %s",
                  full_yaml_path.c_str(), e.what());
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Map metadata: resolution=%.3f, origin=(%.2f, %.2f), height=%d",
              map_resolution_, map_origin_x_, map_origin_y_, map_height_);

  // --- 解析航点 ---
  if (yaml["waypoints"]) {
    for (const auto & wp_node : yaml["waypoints"]) {
      fleet_msgs::msg::Waypoint waypoint;
      waypoint.waypoint_id = wp_node["id"].as<std::string>();
      waypoint.name = wp_node["name"].as<std::string>("");

      // 像素坐标 -> 世界坐标转换
      double pixel_x = wp_node["position"]["x"].as<double>();
      double pixel_y = wp_node["position"]["y"].as<double>();

      waypoint.pose.position.x = pixel_x * map_resolution_ + map_origin_x_;
      if (map_height_ > 0) {
        waypoint.pose.position.y = map_origin_y_ + (map_height_ - pixel_y) * map_resolution_;
      } else {
        waypoint.pose.position.y = pixel_y * map_resolution_ + map_origin_y_;
      }
      waypoint.pose.position.z = wp_node["position"]["z"].as<double>(0.0);
      waypoint.pose.orientation.w = 1.0;

      RCLCPP_INFO(node_->get_logger(),
                   "Waypoint %s: pixel(%.1f, %.1f) -> world(%.2f, %.2f) [map_height=%d]",
                   waypoint.waypoint_id.c_str(), pixel_x, pixel_y,
                   waypoint.pose.position.x, waypoint.pose.position.y, map_height_);

      // 连接关系
      if (wp_node["connections"]) {
        for (const auto & conn : wp_node["connections"]) {
          waypoint.connections.push_back(conn.as<std::string>());
        }
      }

      waypoint.is_parking_spot = wp_node["is_parking_spot"].as<bool>(false);
      waypoint.is_charging_station = wp_node["is_charging_station"].as<bool>(false);
      waypoint.radius = wp_node["radius"].as<float>(0.5);

      current_map_.waypoints.push_back(waypoint);
    }
  }

  // --- 地图元信息 ---
  if (yaml["map_id"]) {
    current_map_.map_id = yaml["map_id"].as<std::string>();
  }
  if (yaml["map_name"]) {
    current_map_.map_name = yaml["map_name"].as<std::string>();
  }
  if (yaml["map_yaml_path"]) {
    current_map_.map_yaml_path = yaml["map_yaml_path"].as<std::string>();
  }
}

YAML::Node TrafficManager::save_to_yaml()
{
  YAML::Node yaml;

  // 地图元信息
  yaml["map_id"] = current_map_.map_id;
  yaml["map_name"] = current_map_.map_name;
  yaml["map_yaml_path"] = current_map_.map_yaml_path;
  yaml["map_resolution"] = map_resolution_;
  yaml["map_origin"] = YAML::Node(YAML::NodeType::Sequence);
  yaml["map_origin"].push_back(map_origin_x_);
  yaml["map_origin"].push_back(map_origin_y_);
  yaml["map_origin"].push_back(0.0);
  if (!map_image_.empty()) {
    yaml["map_image"] = map_image_;
  }
  yaml["waypoints"] = YAML::Node(YAML::NodeType::Sequence);

  // 导出航点（世界坐标 -> 像素坐标逆变换）
  for (const auto & waypoint : current_map_.waypoints) {
    YAML::Node wp_node;
    wp_node["id"] = waypoint.waypoint_id;
    wp_node["name"] = waypoint.name;
    // 注意：交通图 YAML 的 position.x/y 在本工程中定义为像素坐标（load_from_yaml 会再转换到世界坐标）。
    // 这里必须做 world->pixel 的逆变换，保证 save->load 坐标不漂移。
    const double pixel_x = (waypoint.pose.position.x - map_origin_x_) / map_resolution_;
    double pixel_y = (waypoint.pose.position.y - map_origin_y_) / map_resolution_;
    if (map_height_ > 0) {
      pixel_y = static_cast<double>(map_height_) - pixel_y;
    }
    wp_node["position"]["x"] = pixel_x;
    wp_node["position"]["y"] = pixel_y;
    wp_node["position"]["z"] = waypoint.pose.position.z;

    wp_node["connections"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto & conn : waypoint.connections) {
      wp_node["connections"].push_back(conn);
    }

    wp_node["is_parking_spot"] = waypoint.is_parking_spot;
    wp_node["is_charging_station"] = waypoint.is_charging_station;
    wp_node["radius"] = waypoint.radius;

    yaml["waypoints"].push_back(wp_node);
  }

  return yaml;
}


}  // namespace fleet_manager
