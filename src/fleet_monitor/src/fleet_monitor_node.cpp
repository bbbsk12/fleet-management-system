#include "fleet_monitor/fleet_monitor_node.hpp"
#include <regex>
#include <algorithm>

namespace fleet_monitor
{
namespace
{
std::string extract_robot_namespace_by_suffix(const std::string & name, const std::string & suffix)
{
  std::size_t pos = name.rfind(suffix);
  if (pos == std::string::npos) {
    return "";
  }
  std::string robot_ns = name.substr(0, pos);
  if (!robot_ns.empty() && robot_ns[0] == '/') {
    robot_ns = robot_ns.substr(1);
  }
  return robot_ns;
}
}  // namespace

FleetMonitorNode::FleetMonitorNode()
: Node("fleet_monitor"),
  robot_namespace_regex_(R"((/\w+/tf)$)"),
  discover_interval_(std::chrono::seconds(5)),
  last_discover_time_(this->now())  // 使用 ROS 时间初始化
{
  tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 100,
    std::bind(&FleetMonitorNode::tf_callback, this, std::placeholders::_1));
  
  fleet_status_pub_ = this->create_publisher<fleet_msgs::msg::FleetStatus>(
    "/fleet_monitor/fleet_status", 10);
  
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&FleetMonitorNode::timer_callback, this));
  
  get_robot_status_srv_ = this->create_service<fleet_msgs::srv::GetRobotStatus>(
    "~/get_robot_status",
    std::bind(&FleetMonitorNode::handle_get_robot_status, this,
              std::placeholders::_1, std::placeholders::_2));
  
  RCLCPP_INFO(this->get_logger(), "event=fleet_monitor.start component=fleet_monitor reason=NODE_STARTED");
  RCLCPP_INFO(this->get_logger(), "event=fleet_monitor.discovery component=fleet_monitor reason=AUTO_DISCOVER_ENABLED detail=source:/tf");
}

void FleetMonitorNode::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (const auto & transform : msg->transforms) {
    std::string frame_id = transform.header.frame_id;
    std::string child_frame_id = transform.child_frame_id;
    
    // 自动发现所有 base_footprint 的 TF，提取命名空间
    if (child_frame_id.find("base_footprint") != std::string::npos) {
      std::size_t pos = child_frame_id.find("/base_footprint");
      if (pos != std::string::npos) {
        std::string robot_ns = child_frame_id.substr(0, pos);
        // robot_ns 可能带前导 '/', 需要剔除以保证后续话题订阅形如 /<robot_ns>/tf
        if (!robot_ns.empty() && robot_ns[0] == '/') {
          robot_ns = robot_ns.substr(1);
        }
        if (!robot_ns.empty()) {
          get_or_create_monitor(robot_ns);
        }
      }
    }
  }
}

void FleetMonitorNode::timer_callback()
{
  discover_robots();
  update_fleet_status();
}

void FleetMonitorNode::discover_robots()
{
  auto now = this->now();
  if ((now - last_discover_time_).seconds() < discover_interval_.count()) {
    return;
  }
  
  last_discover_time_ = now;
  
  // 通过话题名发现机器人（支持 /<namespace>/tf 格式）
  auto topics = this->get_topic_names_and_types();
  std::regex tf_pattern(R"(/([^/]+)/tf$)");
  
  RCLCPP_DEBUG(this->get_logger(), "Discovering robots, found %zu topics", topics.size());
  
  for (const auto & [topic, types] : topics) {
    std::smatch match;
    // 匹配 /<namespace>/tf 格式
    if (std::regex_search(topic, match, tf_pattern)) {
      std::string robot_ns = match[1].str();
      // 移除可能的前导斜杠
      if (!robot_ns.empty() && robot_ns[0] == '/') {
        robot_ns = robot_ns.substr(1);
      }
      RCLCPP_INFO(this->get_logger(), "event=robot.discovered component=fleet_monitor reason=TOPIC_MATCH detail=topic:%s robot:%s",
                  topic.c_str(), robot_ns.c_str());
      if (!robot_ns.empty()) {
        get_or_create_monitor(robot_ns);
      }
    }
    
    // 匹配 /<namespace>/battery_state 格式
    std::string battery_ns = extract_robot_namespace_by_suffix(topic, "/battery_state");
    if (!battery_ns.empty()) {
      get_or_create_monitor(battery_ns);
    }

    // 匹配 /<namespace>/online_flag 格式（用于心跳发现与在线/离线判定）
    std::string online_ns = extract_robot_namespace_by_suffix(topic, "/online_flag");
    if (!online_ns.empty()) {
      get_or_create_monitor(online_ns);
    }
  }
}

std::shared_ptr<RobotMonitor> FleetMonitorNode::get_or_create_monitor(
  const std::string & robot_namespace)
{
  auto it = robot_monitors_.find(robot_namespace);
  if (it != robot_monitors_.end()) {
    return it->second;
  }
  
  RCLCPP_INFO(this->get_logger(), "event=robot.monitor_create component=fleet_monitor reason=NEW_NAMESPACE robot=%s", robot_namespace.c_str());
  
  auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
  auto monitor = std::make_shared<RobotMonitor>(node_ptr, robot_namespace);
  robot_monitors_[robot_namespace] = monitor;
  
  return monitor;
}

void FleetMonitorNode::update_fleet_status()
{
  fleet_status_.timestamp = this->now();
  fleet_status_.robots.clear();
  fleet_status_.pending_tasks.clear();
  fleet_status_.active_tasks = 0;
  
  size_t online_count = 0;
  size_t offline_count = 0;
  
  for (const auto & [ns, monitor] : robot_monitors_) {
    auto status = monitor->get_status();
    fleet_status_.robots.push_back(status);
    
    if (monitor->is_online()) {
      online_count++;
      if (!status.current_task_id.empty()) {
        fleet_status_.active_tasks++;
      }
    } else {
      offline_count++;
    }
  }
  
  if (offline_count == 0) {
    fleet_status_.system_status = "healthy";
  } else if (online_count > 0) {
    fleet_status_.system_status = "warning";
  } else {
    fleet_status_.system_status = "error";
  }
  
  fleet_status_pub_->publish(fleet_status_);
}

void FleetMonitorNode::handle_get_robot_status(
  const std::shared_ptr<fleet_msgs::srv::GetRobotStatus::Request> request,
  std::shared_ptr<fleet_msgs::srv::GetRobotStatus::Response> response)
{
  response->success = true;
  response->robots.clear();
  
  if (request->robot_id.empty()) {
    for (const auto & [ns, monitor] : robot_monitors_) {
      response->robots.push_back(monitor->get_status());
    }
  } else {
    auto it = robot_monitors_.find(request->robot_id);
    if (it != robot_monitors_.end()) {
      response->robots.push_back(it->second->get_status());
    } else {
      response->success = false;
      response->message = "Robot not found: " + request->robot_id;
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    }
  }
}

}  // namespace fleet_monitor