#include "fleet_manager/task_scheduler.hpp"
#include "fleet_manager/persist_logger.hpp"
#include <chrono>
#include <cmath>
#include <sstream>

namespace fleet_manager
{

namespace
{
bool is_like_executing(const std::string & s)
{
  return s == "assigned" || s == "in_progress" || s == "executing" || s == "waiting_fleet";
}
bool is_finished(const std::string & s)
{
  return s == "completed" || s == "failed" || s == "cancelled";
}
}  // namespace

TaskScheduler::TaskScheduler(rclcpp::Node * node) : node_(node) {}

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

// ==================== Public API ====================

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
  if (!robot_id.empty()) fixed_.insert(t.task_id);

  PersistLogger::log_info("sched.submit", robot_id, t.task_id,
    "wp=" + waypoint_id + " pri=" + std::to_string(priority) +
    " type=" + std::to_string(task_type),
    __FILE__, __LINE__, __func__);

  return t.task_id;
}

std::vector<fleet_msgs::msg::TaskInfo> TaskScheduler::assign_tasks_batch(
  const std::vector<fleet_msgs::msg::RobotStatus> & robots,
  const std::map<std::string, geometry_msgs::msg::Pose> & waypoint_poses)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  std::vector<fleet_msgs::msg::TaskInfo> results;
  if (queue_.empty() || robots.empty()) return results;

  waypoint_poses_ = waypoint_poses;

  // collect available robots
  std::set<std::string> avail;
  for (const auto & r : robots)
    if (r.connection_status == "online") avail.insert(r.robot_id);

  // remove robots that already have active tasks
  for (const auto & [_, t] : all_) {
    if (t.assigned_robot_id.empty()) continue;
    if (is_finished(t.status)) continue;
    if (is_like_executing(t.status)) avail.erase(t.assigned_robot_id);
  }

  // drain queue into pending vector in priority order
  std::vector<fleet_msgs::msg::TaskInfo> pending;
  while (!queue_.empty()) {
    auto t = queue_.top(); queue_.pop();
    auto it = all_.find(t.task_id);
    if (it == all_.end()) continue;
    if (it->second.status == "cancelled") continue;
    if (it->second.status != "pending") continue;
    pending.push_back(it->second);
  }

  // reserve robots for fixed pending tasks
  std::map<std::string, std::string> reserved_by_fixed;
  for (const auto & t : pending) {
    if (t.assigned_robot_id.empty()) continue;
    if (reserved_by_fixed.count(t.assigned_robot_id)) continue;
    reserved_by_fixed[t.assigned_robot_id] = t.task_id;
  }
  for (const auto & [rid, _] : reserved_by_fixed) avail.erase(rid);

  // greedy assignment with FCFS gate
  std::vector<fleet_msgs::msg::TaskInfo> unmatched;
  size_t processed = 0;

  for (size_t i = 0; i < pending.size(); ++i) {
    auto task = pending[i];
    processed = i + 1;

    auto wp_it = waypoint_poses.find(task.waypoint_id);
    if (wp_it == waypoint_poses.end()) { unmatched.push_back(task); continue; }

    std::string best;
    double best_dist = std::numeric_limits<double>::max();

    if (!task.assigned_robot_id.empty()) {
      // fixed robot task
      const auto rsv = reserved_by_fixed.find(task.assigned_robot_id);
      if ((rsv != reserved_by_fixed.end() && rsv->second == task.task_id) ||
          avail.count(task.assigned_robot_id)) {
        best = task.assigned_robot_id;
      }
    } else {
      // nearest available
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
      // fixed tasks don't block the FCFS gate
      if (!task.assigned_robot_id.empty()) continue;

      // FCFS gate with timeout: if same task blocks repeatedly,
      // skip it to avoid starving lower-priority tasks
      auto & skip_count = fcfc_skip_count_[task.task_id];
      skip_count++;
      if (skip_count >= kFcfcGateMaxSkips) {
        PersistLogger::log_warn("sched.fcfc_skip", "", task.task_id,
          "FCFS gate blocked " + std::to_string(skip_count) + " ticks, skipping",
          __FILE__, __LINE__, __func__);
        skip_count = 0;
        continue;  // skip this task, continue to lower-priority
      }
      break;  // FCFS gate
    }

    fcfc_skip_count_.erase(task.task_id);  // reset on successful assignment

    task.assigned_robot_id = best;
    task.status = "assigned";
    task.started_at = node_->now();
    all_[task.task_id] = task;
    results.push_back(task);

    avail.erase(best);
    if (avail.empty()) break;
  }

  // re-queue unmatched
  for (const auto & t : unmatched) queue_.push(t);
  for (size_t i = processed; i < pending.size(); ++i) queue_.push(pending[i]);

  return results;
}

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
  if (!is_fixed) it->second.assigned_robot_id.clear();
  queue_.push(it->second);
}

void TaskScheduler::mark_task_pending_preserve(const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = all_.find(task_id);
  if (it == all_.end()) return;

  remove_from_queue(task_id);
  it->second.status = "pending";
  // keep assigned_robot_id binding (do NOT clear)
  retry_cycle_count_[task_id]++;  // track re-queue count
  queue_.push(it->second);

  PersistLogger::log_info("sched.preserve", it->second.assigned_robot_id, task_id,
    "re-queued preserving robot binding (cycle=" +
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

void TaskScheduler::complete_task(const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  remove_from_queue(task_id);
  auto it = all_.find(task_id);
  if (it != all_.end()) {
    if (is_finished(it->second.status)) return;
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
    // Guard: don't overwrite already-terminal status
    if (is_finished(it->second.status)) return;
    it->second.status = "cancelled";
    fixed_.erase(task_id);
    fcfc_skip_count_.erase(task_id);
    retry_cycle_count_.erase(task_id);
    PersistLogger::log_info("sched.cancel", it->second.assigned_robot_id, task_id,
      "task cancelled", __FILE__, __LINE__, __func__);
  }
}

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

void TaskScheduler::repair_queue()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  // Rebuild queue from scratch: O(n log n) instead of O(n²)
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
