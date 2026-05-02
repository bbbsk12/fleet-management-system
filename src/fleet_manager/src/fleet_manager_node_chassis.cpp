#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#include <tf2/LinearMath/Quaternion.hpp>

namespace fleet_manager
{
// ==================== 底盘任务执行 ====================

uint16_t FleetManagerNode::waypoint_id_to_uint16(const std::string & waypoint_id) const
{
  // 尝试提取数字部分：如 "wp_020" → 20, "20" → 20, "waypoint_5" → 5
  std::string num_str;
  for (auto it = waypoint_id.rbegin(); it != waypoint_id.rend(); ++it) {
    if (std::isdigit(*it)) {
      num_str.insert(num_str.begin(), *it);
    } else if (!num_str.empty()) {
      break;  // 遇到非数字且已收集了数字，停止
    }
  }
  if (num_str.empty()) {
    // 全部非数字，用哈希低16位
    return static_cast<uint16_t>(std::hash<std::string>{}(waypoint_id) & 0xFFFF);
  }
  unsigned long val = std::stoul(num_str);
  return static_cast<uint16_t>(val & 0xFFFF);
}

void FleetManagerNode::on_navigation_succeeded(const std::string & robot_id, const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  const uint8_t task_type = task_scheduler_->get_task_type(task_id);

  if (task_type == 1 || task_type == 0) {
    // CRUISE 或未指定类型 → 导航成功即完成
    task_scheduler_->complete_task(task_id);
    fleet_msgs::msg::TaskInfo ti;
    ti.task_id = task_id;
    ti.status = "completed";
    task_status_pub_->publish(ti);
    finalize_task_completion(robot_id, task_id);
    return;
  }

  // LOAD(2) / UNLOAD(3) / SITE_SPECIFIC(4) → 发送任务给底盘，等反馈
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info) {
    task_scheduler_->fail_task(task_id, "No nav info for chassis task");
    finalize_task_completion(robot_id, task_id);
    return;
  }

  // 获取任务详情
  const auto task_info = task_scheduler_->get_task_info(task_id);
  if (task_info.task_id.empty()) {
    task_scheduler_->fail_task(task_id, "Task info not found for chassis dispatch");
    finalize_task_completion(robot_id, task_id);
    return;
  }

  // 将任务状态设为 executing
  task_scheduler_->mark_task_executing(task_id);
  fleet_msgs::msg::TaskInfo ti;
  ti.task_id = task_id;
  ti.status = "executing";
  task_status_pub_->publish(ti);

  // 发送任务指令给底盘
  const uint16_t wp_id_num = waypoint_id_to_uint16(task_info.waypoint_id);
  send_task_cmd_to_chassis(robot_id, task_id, wp_id_num, task_type, task_info.site_code);
}

void FleetManagerNode::send_task_cmd_to_chassis(
  const std::string & robot_id,
  const std::string & task_id,
  uint16_t waypoint_id_num,
  uint8_t task_type,
  uint32_t site_code)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info || !nav_info->task_cmd_pub) {
    RCLCPP_ERROR(this->get_logger(),
      "Cannot send TaskCmd to robot %s: publisher not available", robot_id.c_str());
    return;
  }

  // 解析 task_id 中的数字作为 uint64
  uint64_t task_id_num = 0;
  try {
    // task_id 格式: "task_<millis>_<counter>"，取哈希低64位
    task_id_num = std::hash<std::string>{}(task_id);
  } catch (...) {
    task_id_num = 0;
  }

  fleet_msgs::msg::TaskCmd cmd;
  cmd.task_id = task_id_num;
  cmd.waypoint_id = waypoint_id_num;
  cmd.task_type = task_type;
  cmd.site_code = site_code;
  cmd.ack = false;  // 新任务指令，不是ACK

  nav_info->task_cmd_pub->publish(cmd);

  // 记录状态
  nav_info->chassis_task_sent = true;
  nav_info->chassis_handshake_ok = false;
  nav_info->chassis_completion_acked = false;
  nav_info->chassis_handshake_deadline = this->now() +
    rclcpp::Duration::from_seconds(chassis_handshake_timeout_sec_);
  nav_info->chassis_exec_deadline = this->now() +
    rclcpp::Duration::from_seconds(chassis_handshake_timeout_sec_ + chassis_exec_timeout_sec_);
  nav_info->pending_task_type = task_type;
  nav_info->pending_site_code = site_code;
  nav_info->pending_waypoint_id_num = waypoint_id_num;
  nav_info->pending_task_id_num = task_id_num;

  // 注意：不清空 current_task_id，等底盘反馈后才清
  // 也不释放占用锁，等底盘反馈后才释放

  RCLCPP_INFO(this->get_logger(),
    "event=chassis.send task=%s robot=%s state_prev=executing state_new=executing reason=DISPATCHED detail=task_num=%lu wp=%u type=%u site=0x%x",
    task_id.c_str(), robot_id.c_str(), task_id_num,
    waypoint_id_num, task_type, site_code);
  PersistLogger::log_info("chassis.send", robot_id, task_id,
    "TaskCmd sent: wp=" + std::to_string(waypoint_id_num) +
    " type=" + std::to_string(task_type) +
    " site=0x" + std::to_string(site_code),
    __FILE__, __LINE__, __func__);
}

void FleetManagerNode::send_chassis_ack(
  const std::string & robot_id,
  uint64_t task_id_num)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info || !nav_info->task_cmd_pub) {
    RCLCPP_WARN(this->get_logger(),
      "Cannot send chassis ACK to robot %s: publisher not available", robot_id.c_str());
    return;
  }

  fleet_msgs::msg::TaskCmd ack;
  ack.task_id = task_id_num;
  ack.ack = true;
  // 其他字段填 0/默认值（底盘收到 ack=true 时应忽略这些字段）
  ack.waypoint_id = 0;
  ack.task_type = 0;
  ack.site_code = 0;

  nav_info->task_cmd_pub->publish(ack);

  RCLCPP_INFO(this->get_logger(),
    "event=chassis.ack task=- robot=%s state_prev=- state_new=- reason=ACK_SENT detail=task_num=%lu",
    robot_id.c_str(), task_id_num);
  PersistLogger::log_info("chassis.ack", robot_id, "",
    "ACK sent for task_num=" + std::to_string(task_id_num),
    __FILE__, __LINE__, __func__);
}

void FleetManagerNode::chassis_feedback_callback(
  const std::string & robot_id,
  const fleet_msgs::msg::TaskFb::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto nav_info = get_robot_nav_info(robot_id);
  const uint8_t status = msg->status;
  const uint64_t fb_task_id_num = msg->task_id;

  // ===== 情况1：底盘重发反馈，但调度已处理 =====
  // 底盘因未收到 ACK 可能重发 TaskFb，此时 chassis_task_sent 可能为 false
  // - chassis_handshake_ok=true + chassis_task_sent=true + status=0: 握手已确认，重发ACK
  // - chassis_completion_acked=true + status=1/2: 完成已确认，重发ACK
  if (!nav_info || !nav_info->chassis_task_sent) {
    // 检查是否是重发的完成/错误反馈
    if (nav_info && nav_info->chassis_completion_acked &&
        (status == 1 || status == 2)) {
      RCLCPP_INFO(this->get_logger(),
        "Chassis resent TaskFb(status=%d), re-sending ACK: robot=%s task_num=%lu",
        status, robot_id.c_str(), fb_task_id_num);
      send_chassis_ack(robot_id, fb_task_id_num);
      return;
    }
    RCLCPP_DEBUG(this->get_logger(),
      "Ignoring TaskFb from %s: no pending chassis task", robot_id.c_str());
    return;
  }

  // ===== 情况2：底盘重发握手（status=0），但调度已经收到过握手 =====
  // 握手ACK可能丢了，底盘重发HANDSHAKE_OK，需要重新回ACK
  if (status == 0 && nav_info->chassis_handshake_ok) {
    RCLCPP_INFO(this->get_logger(),
      "Chassis resent HANDSHAKE_OK, re-sending ACK: robot=%s task_num=%lu",
      robot_id.c_str(), fb_task_id_num);
    send_chassis_ack(robot_id, fb_task_id_num);
    return;
  }

  const std::string task_id = nav_info->current_task_id;
  if (task_id.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "TaskFb from %s but current_task_id is empty", robot_id.c_str());
    return;
  }

  // ===== 防御：校验 task_id 匹配 =====
  // TaskFb 中的 task_id(uint64) 应与调度系统发送的 pending_task_id_num 匹配
  // 不匹配时仍回 ACK（让底盘停止重发），但不处理为当前任务的结果
  if (fb_task_id_num != nav_info->pending_task_id_num) {
    RCLCPP_WARN(this->get_logger(),
      "TaskFb task_id mismatch from %s: fb_task_num=%lu pending_task_num=%lu, "
      "sending ACK but not processing as current task result",
      robot_id.c_str(), fb_task_id_num, nav_info->pending_task_id_num);
    send_chassis_ack(robot_id, fb_task_id_num);
    return;
  }

  switch (status) {
    case 0:  // HANDSHAKE_OK — 底盘确认收到任务，开始执行
    {
      RCLCPP_INFO(this->get_logger(),
        "event=chassis.handshake task=%s robot=%s state_prev=executing state_new=executing reason=HANDSHAKE_OK",
        task_id.c_str(), robot_id.c_str());
      nav_info->chassis_handshake_ok = true;
      nav_info->chassis_retry_count = 0;
      // 回复 ACK，让底盘知道调度收到了握手
      send_chassis_ack(robot_id, fb_task_id_num);
      PersistLogger::log_info("chassis.handshake", robot_id, task_id,
        "Handshake OK, ACK sent", __FILE__, __LINE__, __func__);
      break;
    }

    case 1:  // COMPLETED — 底盘完成执行
    {
      RCLCPP_INFO(this->get_logger(),
        "event=chassis.complete task=%s robot=%s state_prev=executing state_new=completed reason=CHASSIS_COMPLETED",
        task_id.c_str(), robot_id.c_str());
      // 先发 ACK，让底盘停止重发
      send_chassis_ack(robot_id, fb_task_id_num);
      nav_info->chassis_task_sent = false;
      nav_info->chassis_handshake_ok = false;
      nav_info->chassis_retry_count = 0;
      nav_info->chassis_completion_acked = true;
      task_scheduler_->complete_task(task_id);
      fleet_msgs::msg::TaskInfo ti;
      ti.task_id = task_id;
      ti.status = "completed";
      task_status_pub_->publish(ti);
      finalize_task_completion(robot_id, task_id);
      PersistLogger::log_info("chassis.complete", robot_id, task_id,
        "Task completed by chassis, ACK sent", __FILE__, __LINE__, __func__);
      break;
    }

    case 2:  // ERROR — 底盘执行失败
    {
      const uint8_t error_code = msg->error_code;
      const std::string error_msg =
        "Chassis error: code=0x" + std::to_string(error_code);
      RCLCPP_ERROR(this->get_logger(),
        "event=chassis.error task=%s robot=%s state_prev=executing state_new=failed reason=CHASSIS_ERROR detail=error_code=0x%02x",
        task_id.c_str(), robot_id.c_str(), error_code);
      // 先发 ACK，让底盘停止重发
      send_chassis_ack(robot_id, fb_task_id_num);
      nav_info->chassis_task_sent = false;
      nav_info->chassis_handshake_ok = false;
      nav_info->chassis_completion_acked = true;
      task_scheduler_->fail_task(task_id, error_msg);
      fleet_msgs::msg::TaskInfo ti;
      ti.task_id = task_id;
      ti.status = "failed";
      task_status_pub_->publish(ti);
      finalize_task_completion(robot_id, task_id);
      PersistLogger::log_error("chassis.error", robot_id, task_id,
        error_msg + ", ACK sent", __FILE__, __LINE__, __func__);
      break;
    }

    default:
      RCLCPP_WARN(this->get_logger(),
        "Unknown TaskFb status %d from robot %s", status, robot_id.c_str());
      break;
  }
}

void FleetManagerNode::chassis_timeout_check_callback()
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  const auto now = this->now();

  for (auto & [robot_id, nav_info] : robot_nav_info_) {
    if (!nav_info || !nav_info->chassis_task_sent) continue;

    const std::string task_id = nav_info->current_task_id;
    if (task_id.empty()) {
      nav_info->chassis_task_sent = false;
      continue;
    }

    // 握手超时检查
    if (!nav_info->chassis_handshake_ok) {
      if (now > nav_info->chassis_handshake_deadline) {
        nav_info->chassis_retry_count++;
        if (nav_info->chassis_retry_count <= chassis_max_retries_) {
          RCLCPP_WARN(this->get_logger(),
            "event=chassis.timeout task=%s robot=%s state_prev=executing state_new=executing reason=HANDSHAKE_TIMEOUT_RETRY detail=retry=%d/%d",
            task_id.c_str(), robot_id.c_str(),
            nav_info->chassis_retry_count, chassis_max_retries_);
          // 重发 TaskCmd
          send_task_cmd_to_chassis(
            robot_id, task_id,
            nav_info->pending_waypoint_id_num,
            nav_info->pending_task_type,
            nav_info->pending_site_code);
          PersistLogger::log_warn("chassis.timeout", robot_id, task_id,
            "Handshake timeout, retry " + std::to_string(nav_info->chassis_retry_count),
            __FILE__, __LINE__, __func__);
        } else {
          RCLCPP_ERROR(this->get_logger(),
            "event=chassis.timeout task=%s robot=%s state_prev=executing state_new=failed reason=HANDSHAKE_TIMEOUT_EXHAUSTED detail=max_retries=%d",
            task_id.c_str(), robot_id.c_str(), chassis_max_retries_);
          nav_info->chassis_task_sent = false;
          nav_info->chassis_handshake_ok = false;
          nav_info->chassis_retry_count = 0;
          nav_info->chassis_completion_acked = true;
          task_scheduler_->fail_task(task_id, "Chassis handshake timeout");
          fleet_msgs::msg::TaskInfo ti;
          ti.task_id = task_id;
          ti.status = "failed";
          task_status_pub_->publish(ti);
          finalize_task_completion(robot_id, task_id);
          PersistLogger::log_error("chassis.timeout", robot_id, task_id,
            "Handshake timeout exhausted", __FILE__, __LINE__, __func__);
        }
      }
      continue;  // 握手未完成，不检查执行超时
    }

    // 执行超时检查（握手成功后）
    if (now > nav_info->chassis_exec_deadline) {
      RCLCPP_ERROR(this->get_logger(),
        "event=chassis.timeout task=%s robot=%s state_prev=executing state_new=failed reason=EXEC_TIMEOUT detail=timeout_sec=%.1f",
        task_id.c_str(), robot_id.c_str(), chassis_exec_timeout_sec_);
      nav_info->chassis_task_sent = false;
      nav_info->chassis_handshake_ok = false;
      nav_info->chassis_completion_acked = true;
      task_scheduler_->fail_task(task_id, "Chassis execution timeout");
      fleet_msgs::msg::TaskInfo ti;
      ti.task_id = task_id;
      ti.status = "failed";
      task_status_pub_->publish(ti);
      finalize_task_completion(robot_id, task_id);
      PersistLogger::log_error("chassis.timeout", robot_id, task_id,
        "Execution timeout", __FILE__, __LINE__, __func__);
    }
  }
}

bool FleetManagerNode::is_robot_executing(const std::string & robot_id) const
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto it = robot_nav_info_.find(robot_id);
  if (it == robot_nav_info_.end() || !it->second) return false;
  return it->second->chassis_task_sent;
}

void FleetManagerNode::finalize_task_completion(
  const std::string & robot_id,
  const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  (void)task_id;
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info) return;

  // 释放占用/预约锁
  occupancy_manager_->release_reservations(robot_id);

  // 清理导航状态
  nav_info->current_task_id.clear();
  nav_info->has_active_goal = false;
  nav_info->route_waypoints.clear();
  nav_info->current_waypoint_index = 0;
  reset_through_segment_state(nav_info);

  // 清理底盘任务状态
  nav_info->chassis_task_sent = false;
  nav_info->chassis_handshake_ok = false;
  nav_info->chassis_retry_count = 0;
  // 注意：不清空 chassis_completion_acked，因为底盘可能重发 TaskFb，
  // 需要 chassis_completion_acked=true 来触发 ACK 回复
}

}  // namespace fleet_manager
