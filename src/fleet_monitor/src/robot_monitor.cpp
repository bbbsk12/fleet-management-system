// =============================================================================
// @file    robot_monitor.cpp
// @brief   单体机器人监控器实现
//
// 实现机器人的在线状态判定、位姿更新、电池监控和导航任务跟踪。
// 在线判定采用分层策略：
//   1. 若存在 online_flag 心跳话题，则以其作为首要在线依据；
//   2. 否则回退到 TF / BatteryState 话题的超时检测。
// =============================================================================

#include "fleet_monitor/robot_monitor.hpp"
#include <chrono>
#include <thread>

namespace fleet_monitor
{

// =============================================================================
/// @brief 构造函数
///
/// 初始化机器人状态默认值，创建 TF 缓存、各话题订阅器、导航动作客户端
/// 以及周期定时器。
// =============================================================================
RobotMonitor::RobotMonitor(rclcpp::Node::SharedPtr node, const std::string & robot_namespace)
: node_(node),
  robot_namespace_(robot_namespace),
  is_online_(false),
  last_battery_update_(node->now()),
  last_online_flag_update_(node->now()),
  last_tf_update_(node->now()),
  online_timeout_(std::chrono::seconds(5)),
  has_online_flag_(false),
  has_active_goal_(false)
{
  // 初始化机器人状态默认值
  robot_status_.robot_id = robot_namespace_;
  robot_status_.robot_namespace = robot_namespace_;
  robot_status_.connection_status = "offline";
  robot_status_.battery_percentage = 100.0;  // 默认满电
  robot_status_.battery_low = false;
  robot_status_.current_task_id = "";
  robot_status_.nav_status = "idle";

  // 创建 TF 缓存，用于后续坐标变换查询
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

  // 订阅机器人的 TF 话题，用于位姿更新及在线状态判定（降级方案）
  tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/" + robot_namespace + "/tf", 100,
    std::bind(&RobotMonitor::tf_callback, this, std::placeholders::_1));

  // 订阅电池状态话题
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/" + robot_namespace + "/battery_state", 10,
    std::bind(&RobotMonitor::battery_callback, this, std::placeholders::_1));

  // 订阅心跳话题（/<robot_namespace>/online_flag），用于稳定判定在线/离线
  // 解决 zenoh 重启后 connection_status 不更新等场景下的可靠性问题
  online_flag_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "/" + robot_namespace + "/online_flag", 10,
    std::bind(&RobotMonitor::online_flag_callback, this, std::placeholders::_1));

  // 创建导航动作客户端
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_, "/" + robot_namespace + "/navigate_to_pose");

  // 创建周期定时器，驱动在线状态判定及位姿更新
  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&RobotMonitor::timer_callback, this));

  RCLCPP_INFO(
    node_->get_logger(),
    "event=robot.monitor_start robot=%s state_prev=- state_new=offline reason=MONITOR_CREATED",
    robot_namespace_.c_str());
}

// =============================================================================
/// @brief 心跳话题回调（/<robot_namespace>/online_flag）
///
/// 每次收到 Int32 消息（非零值表示在线）时更新心跳时间戳。
/// 若为首条心跳，则将机器人标记为在线。
/// 此回调是"首要在线依据"的数据源。
// =============================================================================
void RobotMonitor::online_flag_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  (void)msg;
  const auto now = node_->now();
  last_online_flag_update_ = now;
  if (!has_online_flag_) {
    has_online_flag_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "event=robot.connection_change robot=%s state_prev=offline state_new=online reason=ONLINE_FLAG_FIRST_SEEN",
      robot_namespace_.c_str());
  }
  if (!is_online_) {
    is_online_ = true;
  }
  robot_status_.connection_status = "online";
}

// =============================================================================
/// @brief 电池状态话题回调
///
/// 更新电量和低电量标志。当 online_flag 心跳尚未启用时，电池消息
/// 可作为在线状态的辅助判定依据。
// =============================================================================
void RobotMonitor::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  robot_status_.battery_percentage = msg->percentage;
  robot_status_.battery_low = msg->percentage < 20.0;
  last_battery_update_ = node_->now();

  // 若尚未启用 online_flag 心跳判定，则通过电池消息将机器人标记为在线（降级方案）
  if (!has_online_flag_ && !is_online_) {
    is_online_ = true;
    robot_status_.connection_status = "online";
    RCLCPP_INFO(
      node_->get_logger(),
      "event=robot.connection_change robot=%s state_prev=offline state_new=online reason=BATTERY_HEARTBEAT",
      robot_namespace_.c_str());
  }
}

// =============================================================================
/// @brief TF 话题回调
///
/// 将收到的 TF 变换存入 tf_buffer，更新最后接收时间戳。
/// 当 online_flag 心跳尚未启用时，TF 消息可作为在线状态的辅助判定依据。
// =============================================================================
void RobotMonitor::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  // 将 TF 消息添加到缓存
  for (const auto & transform : msg->transforms) {
    try {
      tf_buffer_->setTransform(transform, robot_namespace_, false);
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(node_->get_logger(), "TF setTransform failed: %s", ex.what());
    }
  }
  last_tf_update_ = node_->now();

  // 若尚未启用 online_flag 心跳判定，则通过 TF 将机器人标记为在线（降级方案）
  if (!has_online_flag_ && !is_online_) {
    is_online_ = true;
    robot_status_.connection_status = "online";
    RCLCPP_INFO(
      node_->get_logger(),
      "event=robot.connection_change robot=%s state_prev=offline state_new=online reason=TF_HEARTBEAT",
      robot_namespace_.c_str());
  }
}

// =============================================================================
/// @brief 周期定时器回调
///
/// 根据在线判定策略更新在线状态：
///   - 若已启用 online_flag 心跳，则以此为首要依据判定在线/离线；
///   - 否则回退到 TF + Battery 的超时检测。
/// 完成后更新机器人位姿及最后更新时间戳。
// =============================================================================
void RobotMonitor::timer_callback()
{
  auto now = node_->now();

  if (has_online_flag_) {
    // online_flag 心跳是"首要在线依据"
    if ((now - last_online_flag_update_).seconds() > online_timeout_.count()) {
      if (is_online_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "event=robot.connection_change robot=%s state_prev=online state_new=offline reason=ONLINE_FLAG_TIMEOUT",
          robot_namespace_.c_str());
      }
      is_online_ = false;
      robot_status_.connection_status = "offline";
    } else {
      // 心跳在有效期内，保持在线标志
      if (!is_online_) {
        RCLCPP_INFO(
          node_->get_logger(),
          "event=robot.connection_change robot=%s state_prev=offline state_new=online reason=ONLINE_FLAG_RECOVERED",
          robot_namespace_.c_str());
      }
      is_online_ = true;
      robot_status_.connection_status = "online";
    }
  } else {
    // 降级方案：使用 TF / Battery 的超时判定逻辑
    if (is_online_) {
      const bool tf_fresh = (now - last_tf_update_).seconds() <= online_timeout_.count();
      const bool battery_fresh = (now - last_battery_update_).seconds() <= online_timeout_.count();
      if (!tf_fresh && !battery_fresh) {
        is_online_ = false;
        robot_status_.connection_status = "offline";
        RCLCPP_WARN(
          node_->get_logger(),
          "event=robot.connection_change robot=%s state_prev=online state_new=offline reason=TF_AND_BATTERY_TIMEOUT",
          robot_namespace_.c_str());
      }
    }
  }

  update_robot_pose();

  robot_status_.last_update = now;
}

// =============================================================================
/// @brief 更新机器人位姿
///
/// 通过 tf_buffer 查询 map -> base_footprint 的变换，获取当前位姿。
/// 多机器人系统中，底盘 frame 可能为 namespaced（如 robot1/base_footprint）
/// 也可能是全局的 base_footprint。优先尝试 namespaced 版本，失败后回退全局。
// =============================================================================
bool RobotMonitor::update_robot_pose()
{
  try {
    // 优先尝试带命名空间的底盘坐标系，失败后回退全局 base_footprint
    const std::string source_frame = "map";
    const std::string target_frame_namespaced = robot_namespace_ + "/base_footprint";
    const std::string target_frame_global = "base_footprint";

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(source_frame, target_frame_namespaced, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      transform = tf_buffer_->lookupTransform(source_frame, target_frame_global, tf2::TimePointZero);
    }

    robot_status_.current_pose.position.x = transform.transform.translation.x;
    robot_status_.current_pose.position.y = transform.transform.translation.y;
    robot_status_.current_pose.position.z = transform.transform.translation.z;
    robot_status_.current_pose.orientation = transform.transform.rotation;

    last_tf_update_ = node_->now();
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_DEBUG(node_->get_logger(), "TF lookup failed: %s", ex.what());
    return false;
  }
}

// =============================================================================
/// @brief 导航目标响应回调
///
/// 处理导航动作服务器的响应。若目标被拒绝，将导航状态置为失败；
/// 若被接受，则保存目标句柄并将状态更新为移动中。
// =============================================================================
void RobotMonitor::goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    robot_status_.nav_status = "failed";
    has_active_goal_ = false;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    robot_status_.nav_status = "moving";
    current_goal_handle_ = goal_handle;
  }
}

// =============================================================================
/// @brief 导航反馈回调
///
/// 实时更新机器人位姿及导航状态为移动中。
// =============================================================================
void RobotMonitor::feedback_callback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  robot_status_.current_pose = feedback->current_pose.pose;
  robot_status_.nav_status = "moving";
}

// =============================================================================
/// @brief 导航结果回调
///
/// 根据导航执行结果更新导航状态：
///   - SUCCEEDED：到达目标，清除当前任务 ID
///   - ABORTED：导航失败
///   - CANCELED：导航被取消，状态重置为空闲
// =============================================================================
void RobotMonitor::result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
{
  has_active_goal_ = false;
  current_goal_handle_.reset();

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      robot_status_.nav_status = "arrived";
      robot_status_.current_task_id = "";
      RCLCPP_INFO(node_->get_logger(), "Navigation succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      robot_status_.nav_status = "failed";
      RCLCPP_ERROR(node_->get_logger(), "Navigation was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      robot_status_.nav_status = "idle";
      RCLCPP_INFO(node_->get_logger(), "Navigation was canceled");
      break;
    default:
      robot_status_.nav_status = "failed";
      RCLCPP_ERROR(node_->get_logger(), "Unknown navigation result");
      break;
  }
}

}  // namespace fleet_monitor
