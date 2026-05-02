#ifndef FLEET_MONITOR__FLEET_MONITOR_NODE_HPP_
#define FLEET_MONITOR__FLEET_MONITOR_NODE_HPP_

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

class FleetMonitorNode : public rclcpp::Node
{
public:
  FleetMonitorNode();

private:
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  
  void timer_callback();
  
  void handle_get_robot_status(
    const std::shared_ptr<fleet_msgs::srv::GetRobotStatus::Request> request,
    std::shared_ptr<fleet_msgs::srv::GetRobotStatus::Response> response);
  
  void discover_robots();
  
  void update_fleet_status();
  
  std::shared_ptr<RobotMonitor> get_or_create_monitor(const std::string & robot_namespace);

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Publisher<fleet_msgs::msg::FleetStatus>::SharedPtr fleet_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<fleet_msgs::srv::GetRobotStatus>::SharedPtr get_robot_status_srv_;
  
  std::map<std::string, std::shared_ptr<RobotMonitor>> robot_monitors_;
  fleet_msgs::msg::FleetStatus fleet_status_;
  
  std::regex robot_namespace_regex_;
  std::chrono::seconds discover_interval_;
  rclcpp::Time last_discover_time_;
};

}  // namespace fleet_monitor

#endif  // FLEET_MONITOR__FLEET_MONITOR_NODE_HPP_