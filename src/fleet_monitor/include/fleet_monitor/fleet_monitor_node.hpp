#ifndef FLEET_MONITOR__FLEET_MONITOR_NODE_HPP_
#define FLEET_MONITOR__FLEET_MONITOR_NODE_HPP_

// =============================================================================
// 头文件包含
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <fleet_msgs/msg/fleet_status.hpp>
#include <fleet_msgs/msg/robot_status.hpp>
#include <fleet_msgs/srv/get_robot_status.hpp>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <regex>
#include "robot_monitor.hpp"

namespace fleet_monitor
{

// =============================================================================
/// @brief 车队监控节点
///
/// 通过订阅 /tf 话题自动发现机器人命名空间，并为每个机器人创建 RobotMonitor 实例。
/// 定期发布聚合的车队状态（FleetStatus），并提供 GetRobotStatus 服务用于查询单个或
/// 全部机器人的状态。
// =============================================================================
class FleetMonitorNode : public rclcpp::Node
{
public:
  /// @brief 构造函数，初始化订阅、发布器、定时器及服务端
  FleetMonitorNode();

private:
  // ===========================================================================
  // 回调函数
  // ===========================================================================

  /// @brief /tf 话题回调，解析 TF 变换并自动发现机器人
  /// @param msg TF 消息，包含多个变换
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  /// @brief 周期定时器回调，执行机器人发现及车队状态更新
  void timer_callback();

  /// @brief 处理 GetRobotStatus 服务请求，返回指定（或全部）机器人的状态
  /// @param request  服务请求，包含可选的 robot_id
  /// @param response 服务响应，包含机器人状态列表
  void handle_get_robot_status(
    const std::shared_ptr<fleet_msgs::srv::GetRobotStatus::Request> request,
    std::shared_ptr<fleet_msgs::srv::GetRobotStatus::Response> response);

  // ===========================================================================
  // 内部逻辑
  // ===========================================================================

  /// @brief 扫描已发布的话题（/namespace/tf 等格式），发现新的机器人
  void discover_robots();

  /// @brief 汇总所有机器人的状态，发布车队整体状态
  void update_fleet_status();

  /// @brief 获取或创建指定命名空间对应的 RobotMonitor 实例
  /// @param robot_namespace 机器人的 ROS 命名空间
  /// @return RobotMonitor 共享指针
  std::shared_ptr<RobotMonitor> get_or_create_monitor(const std::string & robot_namespace);

  // ===========================================================================
  // ROS 通信成员
  // ===========================================================================

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;           ///< /tf 话题订阅器
  rclcpp::Publisher<fleet_msgs::msg::FleetStatus>::SharedPtr fleet_status_pub_; ///< 车队状态发布器
  rclcpp::TimerBase::SharedPtr timer_;                                          ///< 周期定时器
  rclcpp::Service<fleet_msgs::srv::GetRobotStatus>::SharedPtr get_robot_status_srv_; ///< 机器人状态查询服务

  // ===========================================================================
  // 数据成员
  // ===========================================================================

  std::map<std::string, std::shared_ptr<RobotMonitor>> robot_monitors_; ///< 机器人命名空间 -> RobotMonitor 映射表
  fleet_msgs::msg::FleetStatus fleet_status_;                           ///< 缓存的车队状态

  std::regex robot_namespace_regex_;            ///< 用于识别机器人命名空间的正则表达式
  std::chrono::seconds discover_interval_;      ///< 两次发现操作之间的最小时间间隔
  rclcpp::Time last_discover_time_;             ///< 上次执行发现操作的时间戳
};

}  // namespace fleet_monitor

#endif  // FLEET_MONITOR__FLEET_MONITOR_NODE_HPP_
