#ifndef FLEET_MONITOR__ROBOT_MONITOR_HPP_
#define FLEET_MONITOR__ROBOT_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fleet_msgs/msg/robot_status.hpp>
#include <memory>
#include <string>
#include <chrono>

namespace fleet_monitor
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class RobotMonitor
{
public:
  explicit RobotMonitor(rclcpp::Node::SharedPtr node, const std::string & robot_namespace);
  
  fleet_msgs::msg::RobotStatus get_status() const { return robot_status_; }
  
  bool is_online() const { return is_online_; }
  
  std::string get_namespace() const { return robot_namespace_; }

private:
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  // 心跳：每次收到 /<robot_namespace>/online_flag=1，更新在线时间戳
  void online_flag_callback(const std_msgs::msg::Int32::SharedPtr msg);
  
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  
  void timer_callback();
  
  void goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future);
  
  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  
  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);
  
  bool update_robot_pose();

  rclcpp::Node::SharedPtr node_;
  std::string robot_namespace_;
  bool is_online_;
  fleet_msgs::msg::RobotStatus robot_status_;
  
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr online_flag_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  
  rclcpp::Time last_battery_update_;
  rclcpp::Time last_online_flag_update_;
  rclcpp::Time last_tf_update_;
  std::chrono::seconds online_timeout_;

  bool has_online_flag_;
  
  bool has_active_goal_;
  GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
};

}  // namespace fleet_monitor

#endif  // FLEET_MONITOR__ROBOT_MONITOR_HPP_
