#include "fleet_monitor/robot_monitor.hpp"
#include <chrono>
#include <thread>

namespace fleet_monitor
{

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
  robot_status_.robot_id = robot_namespace_;
  robot_status_.robot_namespace = robot_namespace_;
  robot_status_.connection_status = "offline";
  robot_status_.battery_percentage = 100.0;  // 默认满电
  robot_status_.battery_low = false;
  robot_status_.current_task_id = "";
  robot_status_.nav_status = "idle";
  
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  
  // 订阅机器人的 TF 话题
  tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    "/" + robot_namespace + "/tf", 100,
    std::bind(&RobotMonitor::tf_callback, this, std::placeholders::_1));
  
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/" + robot_namespace + "/battery_state", 10,
    std::bind(&RobotMonitor::battery_callback, this, std::placeholders::_1));

  // 心跳：用于稳定判定在线/离线（解决 zenoh 重启后 connection_status 不更新的问题）
  online_flag_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "/" + robot_namespace + "/online_flag", 10,
    std::bind(&RobotMonitor::online_flag_callback, this, std::placeholders::_1));
  
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_, "/" + robot_namespace + "/navigate_to_pose");
  
  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&RobotMonitor::timer_callback, this));
  
  RCLCPP_INFO(
    node_->get_logger(),
    "event=robot.monitor_start robot=%s state_prev=- state_new=offline reason=MONITOR_CREATED",
    robot_namespace_.c_str());
}

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

void RobotMonitor::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  robot_status_.battery_percentage = msg->percentage;
  robot_status_.battery_low = msg->percentage < 20.0;
  last_battery_update_ = node_->now();
  
  // 若已经启用 online_flag 心跳判定，则 battery 不再负责在线/离线切换
  if (!has_online_flag_ && !is_online_) {
    is_online_ = true;
    robot_status_.connection_status = "online";
    RCLCPP_INFO(
      node_->get_logger(),
      "event=robot.connection_change robot=%s state_prev=offline state_new=online reason=BATTERY_HEARTBEAT",
      robot_namespace_.c_str());
  }
}

void RobotMonitor::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  // 将 TF 消息添加到 buffer
  for (const auto & transform : msg->transforms) {
    try {
      tf_buffer_->setTransform(transform, robot_namespace_, false);
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(node_->get_logger(), "TF setTransform failed: %s", ex.what());
    }
  }
  last_tf_update_ = node_->now();
  
  // 若已经启用 online_flag 心跳判定，则 TF 不再负责在线/离线切换
  if (!has_online_flag_ && !is_online_) {
    is_online_ = true;
    robot_status_.connection_status = "online";
    RCLCPP_INFO(
      node_->get_logger(),
      "event=robot.connection_change robot=%s state_prev=offline state_new=online reason=TF_HEARTBEAT",
      robot_namespace_.c_str());
  }
}

void RobotMonitor::timer_callback()
{
  auto now = node_->now();

  if (has_online_flag_) {
    // online_flag 心跳是“首要在线依据”
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
      // 心跳在，保持在线标志
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
    // 回退：旧逻辑仍然使用 TF/Battery 判定在线/离线
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

bool RobotMonitor::update_robot_pose()
{
  try {
    // 多机器人系统中，底盘 frame 可能是 namespaced（如 robot1/base_footprint），
    // 也可能是全局的 base_footprint。优先尝试 namespaced，失败再回退全局。
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

void RobotMonitor::feedback_callback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  robot_status_.current_pose = feedback->current_pose.pose;
  robot_status_.nav_status = "moving";
}

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
