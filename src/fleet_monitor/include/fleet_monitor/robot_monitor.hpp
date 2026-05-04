#ifndef FLEET_MONITOR__ROBOT_MONITOR_HPP_
#define FLEET_MONITOR__ROBOT_MONITOR_HPP_

// =============================================================================
// 头文件包含
// =============================================================================

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

// =============================================================================
/// @brief 单体机器人监控器
///
/// 负责监控单个机器人的在线状态、位姿、电池电量及导航任务状态。
/// 在线判定策略：
///   - 若存在 online_flag 话题，则以其心跳作为首要在线依据；
///   - 否则回退到 TF / BatteryState 话题的超时检测作为降级方案。
// =============================================================================
class RobotMonitor
{
public:
  /// @brief 构造函数
  /// @param node            ROS 节点共享指针，用于创建订阅器、发布器等
  /// @param robot_namespace 该机器人的 ROS 命名空间
  explicit RobotMonitor(rclcpp::Node::SharedPtr node, const std::string & robot_namespace);

  /// @brief 获取当前 RobotStatus（包含位姿、电池、导航状态等信息）
  fleet_msgs::msg::RobotStatus get_status() const { return robot_status_; }

  /// @brief 获取机器人是否在线
  bool is_online() const { return is_online_; }

  /// @brief 获取机器人的命名空间
  std::string get_namespace() const { return robot_namespace_; }

private:
  // ===========================================================================
  // 回调函数
  // ===========================================================================

  /// @brief 电池状态话题回调，更新电量信息
  /// @param msg 电池状态消息
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  /// @brief 心跳话题回调（/<robot_namespace>/online_flag），更新在线时间戳
  /// @details 每次收到在线标志（值为 1）时记录当前时间，用于后续超时判定
  /// @param msg Int32 消息，非零值表示机器人在线
  void online_flag_callback(const std_msgs::msg::Int32::SharedPtr msg);

  /// @brief TF 话题回调，将变换存入 tf_buffer 并更新最后接收时间
  /// @param msg TF 消息，包含机器人坐标系变换
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  /// @brief 周期定时器回调，执行在线状态判定及位姿更新
  void timer_callback();

  /// @brief 导航目标响应回调
  /// @param future 异步获取的 GoalHandle
  void goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future);

  /// @brief 导航反馈回调，实时更新机器人位姿
  /// @param feedback 导航反馈数据
  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  /// @brief 导航结果回调，更新导航状态（到达/中止/取消）
  /// @param result 导航执行结果
  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

  // ===========================================================================
  // 内部逻辑
  // ===========================================================================

  /// @brief 通过 tf_buffer 查询 map -> base_footprint 变换，更新机器人位姿
  /// @return 位姿更新是否成功
  bool update_robot_pose();

  // ===========================================================================
  // ROS 通信及数据成员
  // ===========================================================================

  rclcpp::Node::SharedPtr node_;                    ///< ROS 节点共享指针
  std::string robot_namespace_;                     ///< 机器人命名空间
  bool is_online_;                                  ///< 在线标志
  fleet_msgs::msg::RobotStatus robot_status_;       ///< 聚合的机器人状态

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;      ///< 电池状态订阅器
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr online_flag_sub_;             ///< 在线标志订阅器（心跳）
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;                  ///< TF 订阅器
  rclcpp::TimerBase::SharedPtr timer_;                                                ///< 周期定时器

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;                       ///< 导航动作客户端

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                                        ///< TF 缓存，用于坐标变换查询

  rclcpp::Time last_battery_update_;      ///< 最近一次电池状态更新的时间戳
  rclcpp::Time last_online_flag_update_;  ///< 最近一次心跳标志更新的时间戳
  rclcpp::Time last_tf_update_;           ///< 最近一次 TF 更新的时间戳
  std::chrono::seconds online_timeout_;   ///< 在线超时阈值（超过此时间无心跳则判定离线）

  bool has_online_flag_;                  ///< 是否已收到过 online_flag 心跳（决定在线判定策略）

  bool has_active_goal_;                                    ///< 当前是否有活跃的导航目标
  GoalHandleNavigateToPose::SharedPtr current_goal_handle_; ///< 当前导航目标的句柄
};

}  // namespace fleet_monitor

#endif  // FLEET_MONITOR__ROBOT_MONITOR_HPP_
