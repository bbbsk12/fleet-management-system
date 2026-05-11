#include "fleet_manager/task_scheduler.hpp"
#include "fleet_manager/persist_logger.hpp"
#include <chrono>
#include <cmath>
#include <set>
#include <sstream>

namespace fleet_manager
{

// ============================================================================
// 内部状态判定
// ============================================================================

namespace
{
/// 是否处于执行类状态(活跃但未终结)
bool is_like_executing(const std::string & s)
{
  return s == "assigned" || s == "in_progress" || s == "executing" || s == "waiting_fleet";
}

/// 是否已终结
bool is_finished(const std::string & s)
{
  return s == "completed" || s == "failed" || s == "cancelled";
}
}  // namespace

TaskScheduler::TaskScheduler(rclcpp::Node * node) : node_(node) {}

// ============================================================================
// 内部工具
// ============================================================================

void TaskScheduler::remove_from_queue(const std::string & task_id)
{
  decltype(queue_) rebuilt;
  while (!queue_.empty()) {
    auto t = queue_.top(); queue_.pop();
    if (t.task_id != task_id) rebuilt.push(t);
  }
  queue_.swap(rebuilt);
}

std::string TaskScheduler::generate_id()
{
  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  return "task_" + std::to_string(ms) + "_" + std::to_string(counter_++);
}

// ============================================================================
// 任务提交
// ============================================================================

std::string TaskScheduler::submit_task(
  const std::string & waypoint_id, int priority,
  const std::string & robot_id, uint8_t task_type, uint32_t site_code)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  fleet_msgs::msg::TaskInfo t;
  t.task_id      = generate_id();
  t.waypoint_id  = waypoint_id;
  t.priority     = priority;
  t.task_type    = task_type;
  t.site_code    = site_code;
  t.assigned_robot_id = robot_id;
  t.status       = "pending";
  t.created_at   = node_->now();
  t.started_at   = node_->now();

  queue_.push(t);
  all_[t.task_id] = t;
  if (!robot_id.empty()) fixed_.insert(t.task_id);  // 指定了底盘 → 固定分配

  PersistLogger::log_info("sched.submit", robot_id, t.task_id,
    "wp=" + waypoint_id + " pri=" + std::to_string(priority) +
    " type=" + std::to_string(task_type),
    __FILE__, __LINE__, __func__);

  return t.task_id;
}

// ============================================================================
// 批量分配 — 优先级队列 + 最近距离贪心 + FCFS 门控(30s 超时)
// ============================================================================

std::vector<fleet_msgs::msg::TaskInfo> TaskScheduler::assign_tasks_batch(
  const std::vector<fleet_msgs::msg::RobotStatus> & robots,
  const std::map<std::string, geometry_msgs::msg::Pose> & waypoint_poses)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  std::vector<fleet_msgs::msg::TaskInfo> results;
  if (queue_.empty() || robots.empty()) return results;

  waypoint_poses_ = waypoint_poses;

  // 收集在线底盘
  std::set<std::string> avail;
  for (const auto & r : robots)
    if (r.connection_status == "online") avail.insert(r.robot_id);

  // 排除已有未完成任务的底盘(一个底盘同时只挂起一个任务)
  for (const auto & [_, t] : all_) {
    if (t.assigned_robot_id.empty()) continue;
    if (is_finished(t.status)) continue;
    avail.erase(t.assigned_robot_id);
  }

  // 将队列按优先级弹出到 pending 向量
  std::vector<fleet_msgs::msg::TaskInfo> pending;
  while (!queue_.empty()) {
    auto t = queue_.top(); queue_.pop();
    auto it = all_.find(t.task_id);
    if (it == all_.end()) continue;
    if (it->second.status == "cancelled") continue;
    if (it->second.status != "pending") continue;
    pending.push_back(it->second);
  }

  // fixed 任务预留底盘(先到先得)
  std::map<std::string, std::string> reserved_by_fixed;
  for (const auto & t : pending) {
    if (t.assigned_robot_id.empty()) continue;
    if (reserved_by_fixed.count(t.assigned_robot_id)) continue;
    reserved_by_fixed[t.assigned_robot_id] = t.task_id;
  }
  for (const auto & [rid, _] : reserved_by_fixed) avail.erase(rid);

  // 贪心分配 + FCFS 门控
  std::vector<fleet_msgs::msg::TaskInfo> unmatched;
  size_t processed = 0;

  // M3: 同 target 去重 — 同一批分配里同一 waypoint_id 只允许一个任务出去,
  // 其余保持 pending(由 unmatched 重新入队),避免多个机器人同时去抢一个 target
  // 触发的级联 target_active_defer 与 route 等待环。
  std::set<std::string> targets_claimed_this_batch;

  for (size_t i = 0; i < pending.size(); ++i) {
    auto task = pending[i];
    processed = i + 1;

    // 跳过不存在于交通图的航点
    auto wp_it = waypoint_poses.find(task.waypoint_id);
    if (wp_it == waypoint_poses.end()) { unmatched.push_back(task); continue; }

    // M3: 本批该 target 已被一个高优任务占用 → 这个任务回 pending,继续看下一个候选
    if (targets_claimed_this_batch.count(task.waypoint_id)) {
      unmatched.push_back(task);
      continue;
    }

    std::string best;
    double best_dist = std::numeric_limits<double>::max();

    if (!task.assigned_robot_id.empty()) {
      // fixed 任务: 检查指定底盘是否可用或被预留
      const auto rsv = reserved_by_fixed.find(task.assigned_robot_id);
      if ((rsv != reserved_by_fixed.end() && rsv->second == task.task_id) ||
          avail.count(task.assigned_robot_id)) {
        best = task.assigned_robot_id;
      }
    } else {
      // 普通任务: 选距离目标最近的可用底盘
      for (const auto & r : robots) {
        if (!avail.count(r.robot_id)) continue;
        double d = std::hypot(
          r.current_pose.position.x - wp_it->second.position.x,
          r.current_pose.position.y - wp_it->second.position.y);
        if (d < best_dist) { best_dist = d; best = r.robot_id; }
      }
    }

    if (best.empty()) {
      unmatched.push_back(task);
      // fixed 任务不阻塞 FCFS 门控(等指定底盘空闲)
      if (!task.assigned_robot_id.empty()) continue;

      // FCFS 门控超时: 连续阻塞 > 30 ticks 则跳过
      auto & skip_count = fcfc_skip_count_[task.task_id];
      skip_count++;
      if (skip_count >= kFcfcGateMaxSkips) {
        if (skip_count == kFcfcGateMaxSkips) {
          PersistLogger::log_warn("sched.fcfc_skip", "", task.task_id,
            "FCFS gate blocked " + std::to_string(skip_count) + " ticks, skipping",
            __FILE__, __LINE__, __func__);
        }
        continue;
      }
      break;
    }

    fcfc_skip_count_.erase(task.task_id);  // 分配成功 → 重置门控计数

    task.assigned_robot_id = best;
    task.status = "assigned";
    task.started_at = node_->now();
    all_[task.task_id] = task;
    results.push_back(task);

    targets_claimed_this_batch.insert(task.waypoint_id);
    avail.erase(best);
    if (avail.empty()) break;
  }

  // 未分配的任务重新入队
  for (const auto & t : unmatched) queue_.push(t);
  for (size_t i = processed; i < pending.size(); ++i) queue_.push(pending[i]);

  return results;
}

// ============================================================================
// 状态转换
// ============================================================================

void TaskScheduler::mark_task_navigating(const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = all_.find(task_id);
  if (it == all_.end() || is_finished(it->second.status)) return;
  if (it->second.status == "in_progress") return;
  it->second.status = "in_progress";
  PersistLogger::log_info("sched.navigating", it->second.assigned_robot_id, task_id,
    "task navigating", __FILE__, __LINE__, __func__);
}

void TaskScheduler::mark_task_executing(const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = all_.find(task_id);
  if (it == all_.end() || is_finished(it->second.status)) return;
  it->second.status = "executing";
  PersistLogger::log_info("sched.executing", it->second.assigned_robot_id, task_id,
    "task executing", __FILE__, __LINE__, __func__);
}

void TaskScheduler::mark_task_waiting(const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = all_.find(task_id);
  if (it == all_.end() || is_finished(it->second.status)) return;
  it->second.status = "waiting_fleet";
}

void TaskScheduler::mark_task_pending(const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = all_.find(task_id);
  if (it == all_.end()) return;
  const bool is_fixed = fixed_.count(task_id);
  remove_from_queue(task_id);
  it->second.status = "pending";
  if (!is_fixed) it->second.assigned_robot_id.clear();  // 非 fixed 任务释放底盘绑定
  queue_.push(it->second);
}

void TaskScheduler::mark_task_pending_preserve(const std::string & task_id)
{
  mark_task_pending_retry(task_id, true);
}

void TaskScheduler::mark_task_pending_retry(const std::string & task_id, bool preserve_binding)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = all_.find(task_id);
  if (it == all_.end()) return;

  const bool keep_binding = preserve_binding || fixed_.count(task_id);
  const std::string previous_robot = it->second.assigned_robot_id;
  remove_from_queue(task_id);
  it->second.status = "pending";
  if (!keep_binding) it->second.assigned_robot_id.clear();
  retry_cycle_count_[task_id]++;

  queue_.push(it->second);

  PersistLogger::log_info("sched.retry", previous_robot, task_id,
    "re-queued after retryable failure (cycle=" +
    std::to_string(retry_cycle_count_[task_id]) + ")",
    __FILE__, __LINE__, __func__);
}

bool TaskScheduler::would_exceed_retry_cycles(const std::string & task_id, int max_cycles) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = retry_cycle_count_.find(task_id);
  return (it != retry_cycle_count_.end() && it->second >= max_cycles);
}

int TaskScheduler::retry_cycle_count(const std::string & task_id) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = retry_cycle_count_.find(task_id);
  return (it != retry_cycle_count_.end()) ? it->second : 0;
}

// ============================================================================
// 终结操作(带终态保护)
// ============================================================================

void TaskScheduler::complete_task(const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  remove_from_queue(task_id);
  auto it = all_.find(task_id);
  if (it != all_.end()) {
    if (is_finished(it->second.status)) return;  // 拒绝覆盖已终结状态
    it->second.status = "completed";
    it->second.completed_at = node_->now();
    retry_cycle_count_.erase(task_id);
    PersistLogger::log_info("sched.complete", it->second.assigned_robot_id, task_id,
      "task completed", __FILE__, __LINE__, __func__);
  }
}

void TaskScheduler::fail_task(const std::string & task_id, const std::string & reason)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  remove_from_queue(task_id);
  auto it = all_.find(task_id);
  if (it != all_.end()) {
    if (is_finished(it->second.status)) return;
    it->second.status = "failed";
    fcfc_skip_count_.erase(task_id);
    retry_cycle_count_.erase(task_id);
    PersistLogger::log_error("sched.fail", it->second.assigned_robot_id, task_id,
      reason, __FILE__, __LINE__, __func__);
  }
}

void TaskScheduler::cancel_task(const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  remove_from_queue(task_id);
  auto it = all_.find(task_id);
  if (it != all_.end()) {
    if (is_finished(it->second.status)) return;
    it->second.status = "cancelled";
    fixed_.erase(task_id);
    fcfc_skip_count_.erase(task_id);
    retry_cycle_count_.erase(task_id);
    PersistLogger::log_info("sched.cancel", it->second.assigned_robot_id, task_id,
      "task cancelled", __FILE__, __LINE__, __func__);
  }
}

// ============================================================================
// 查询
// ============================================================================

uint8_t TaskScheduler::get_task_type(const std::string & task_id) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = all_.find(task_id);
  return (it != all_.end()) ? it->second.task_type : 0;
}

fleet_msgs::msg::TaskInfo TaskScheduler::get_task_info(const std::string & task_id) const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = all_.find(task_id);
  return (it != all_.end()) ? it->second : fleet_msgs::msg::TaskInfo{};
}

std::vector<fleet_msgs::msg::TaskInfo> TaskScheduler::get_all_tasks() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  std::vector<fleet_msgs::msg::TaskInfo> v;
  v.reserve(all_.size());
  for (const auto & [_, t] : all_) v.push_back(t);
  return v;
}

size_t TaskScheduler::pending_count() const
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  size_t n = 0;
  for (const auto & [_, t] : all_)
    if (t.status == "pending") ++n;
  return n;
}

// ============================================================================
// 维护
// ============================================================================

void TaskScheduler::repair_queue()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // 从 all_ 全量重建队列 O(n log n)
  decltype(queue_) fresh;
  for (const auto & [id, t] : all_) {
    if (t.status == "pending") fresh.push(t);
  }
  queue_.swap(fresh);
}

void TaskScheduler::purge_finished(size_t max_keep)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  std::vector<std::pair<std::string, rclcpp::Time>> fin;
  for (const auto & [id, t] : all_)
    if (is_finished(t.status)) fin.push_back({id, t.completed_at});

  if (fin.size() <= max_keep) return;

  // 保留最新的 max_keep 个，删除更老的
  std::sort(fin.begin(), fin.end(),
    [](auto & a, auto & b) { return a.second > b.second; });

  for (size_t i = max_keep; i < fin.size(); ++i) {
    all_.erase(fin[i].first);
    fixed_.erase(fin[i].first);
    fcfc_skip_count_.erase(fin[i].first);
    retry_cycle_count_.erase(fin[i].first);
  }
}

}  // namespace fleet_manager
