// =============================================================================
// @file    fleet_monitor_node.cpp
// @brief   车队监控节点实现
//
// 通过解析 /tf 话题自动发现机器人命名空间，为每个机器人创建 RobotMonitor。
// 周期性发布 FleetStatus 聚合消息，并提供 GetRobotStatus 服务接口。
// =============================================================================

#include "fleet_monitor/fleet_monitor_node.hpp"
#include <regex>
#include <algorithm>

namespace fleet_monitor
{
namespace
{

// =============================================================================
/// @brief 通过话题名称的后缀提取机器人命名空间
/// @param name   话题全名（如 /robot1/battery_state）
/// @param suffix 后缀关键词（如 /battery_state）
/// @return 提取出的机器人命名空间，失败返回空字符串
// =============================================================================
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

// =============================================================================
/// @brief 构造函数
///
/// 初始化 TF 订阅器、车队状态发布器、周期定时器以及机器人状态查询服务。
// =============================================================================
FleetMonitorNode::FleetMonitorNode()
: Node("fleet_monitor"),
  robot_namespace_regex_(R"((/\w+/tf)$)"),
  discover_interval_(std::chrono::seconds(5)),
  last_discover_time_(this->now())      // 使用 ROS 时间初始化
{
  // 创建 /tf 话题订阅器，用于自动发现机器人
  tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 100,
    std::bind(&FleetMonitorNode::tf_callback, this, std::placeholders::_1));

  // 创建车队状态发布器
  fleet_status_pub_ = this->create_publisher<fleet_msgs::msg::FleetStatus>(
    "/fleet_monitor/fleet_status", 10);

  // 创建周期定时器，驱动机器人发现及状态更新
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&FleetMonitorNode::timer_callback, this));

  // 创建机器人状态查询服务
  get_robot_status_srv_ = this->create_service<fleet_msgs::srv::GetRobotStatus>(
    "~/get_robot_status",
    std::bind(&FleetMonitorNode::handle_get_robot_status, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "event=fleet_monitor.start component=fleet_monitor reason=NODE_STARTED");
  RCLCPP_INFO(this->get_logger(), "event=fleet_monitor.discovery component=fleet_monitor reason=AUTO_DISCOVER_ENABLED detail=source:/tf");
}

// =============================================================================
/// @brief /tf 话题回调
///
/// 遍历 TF 消息中的所有变换，检测包含 "base_footprint" 的子坐标系，
/// 从而自动发现并创建对应命名空间的 RobotMonitor。
// =============================================================================
void FleetMonitorNode::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (const auto & transform : msg->transforms) {
    std::string frame_id = transform.header.frame_id;
    std::string child_frame_id = transform.child_frame_id;

    // 检测 base_footprint 子坐标系，提取机器人命名空间
    if (child_frame_id.find("base_footprint") != std::string::npos) {
      std::size_t pos = child_frame_id.find("/base_footprint");
      if (pos != std::string::npos) {
        std::string robot_ns = child_frame_id.substr(0, pos);
        // 剔除可能的前导 '/'，确保后续话题订阅格式统一为 /<robot_ns>/tf
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

// =============================================================================
/// @brief 周期定时器回调
///
/// 顺序执行机器人发现与车队状态更新。
// =============================================================================
void FleetMonitorNode::timer_callback()
{
  discover_robots();
  update_fleet_status();
}

// =============================================================================
/// @brief 机器人发现
///
/// 周期性扫描当前 ROS 网络中的话题列表，匹配以下三种模式的命名空间：
///   - /<namespace>/tf
///   - /<namespace>/battery_state
///   - /<namespace>/online_flag
/// 其中 online_flag 话题的发现优先级最高，用于稳定判定在线/离线。
// =============================================================================
void FleetMonitorNode::discover_robots()
{
  auto now = this->now();
  if ((now - last_discover_time_).seconds() < discover_interval_.count()) {
    return;
  }

  last_discover_time_ = now;

  // 扫描话题列表，支持 /<namespace>/tf 格式的自动发现
  auto topics = this->get_topic_names_and_types();
  std::regex tf_pattern(R"(/([^/]+)/tf$)");

  RCLCPP_DEBUG(this->get_logger(), "Discovering robots, found %zu topics", topics.size());

  for (const auto & [topic, types] : topics) {
    std::smatch match;
    // 匹配 /<namespace>/tf 格式的话题
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

    // 匹配 /<namespace>/battery_state 格式的话题
    std::string battery_ns = extract_robot_namespace_by_suffix(topic, "/battery_state");
    if (!battery_ns.empty()) {
      get_or_create_monitor(battery_ns);
    }

    // 匹配 /<namespace>/online_flag 格式的话题（用于心跳发现与在线/离线判定）
    std::string online_ns = extract_robot_namespace_by_suffix(topic, "/online_flag");
    if (!online_ns.empty()) {
      get_or_create_monitor(online_ns);
    }
  }
}

// =============================================================================
/// @brief 获取或创建 RobotMonitor
///
/// 若指定命名空间已有对应的 RobotMonitor，则直接返回；
/// 否则创建新的实例并加入映射表。
// =============================================================================
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

// =============================================================================
/// @brief 更新车队状态
///
/// 遍历所有 RobotMonitor，汇总每个机器人的状态并统计在线/离线数量，
/// 根据整体健康状况设置 system_status，最后发布 FleetStatus 消息。
// =============================================================================
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

  // 根据在线/离线分布设定系统状态
  if (offline_count == 0) {
    fleet_status_.system_status = "healthy";
  } else if (online_count > 0) {
    fleet_status_.system_status = "warning";
  } else {
    fleet_status_.system_status = "error";
  }

  fleet_status_pub_->publish(fleet_status_);
}

// =============================================================================
/// @brief GetRobotStatus 服务处理
///
/// 若请求中指定了 robot_id，则返回对应机器人的状态；否则返回全部机器人的状态。
// =============================================================================
void FleetMonitorNode::handle_get_robot_status(
  const std::shared_ptr<fleet_msgs::srv::GetRobotStatus::Request> request,
  std::shared_ptr<fleet_msgs::srv::GetRobotStatus::Response> response)
{
  response->success = true;
  response->robots.clear();

  if (request->robot_id.empty()) {
    // 请求未指定 robot_id，返回所有机器人的状态
    for (const auto & [ns, monitor] : robot_monitors_) {
      response->robots.push_back(monitor->get_status());
    }
  } else {
    // 查找指定 robot_id 对应的机器人
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
