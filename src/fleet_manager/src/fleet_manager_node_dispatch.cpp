#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#
#include <algorithm>
#
namespace fleet_manager
{
namespace
{
constexpr int kPriHold = 10;
constexpr int kPriAvoidance = 20;
constexpr int kPriNav = 30;
constexpr int kPriGlobal = 0;
constexpr const char * kGlobalKey = "__global__";
constexpr const char * kGlobalHoldKey = "__global__.hold_sm";
constexpr const char * kGlobalPublishKey = "__global__.publish_status";
}
#
void FleetManagerNode::enqueue_robot_cancel(
  const std::string & robot_id, const std::string & tag, std::function<void()> fn)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto & pd = pending_dispatch_by_robot_[robot_id];
  pd.cancel_fn = std::move(fn);
  pd.updated_at = this->now();
  pd.cancel_executed_this_tick = false;
  PersistLogger::log_info(
    "dispatch.enqueue_cancel", robot_id, "",
    "tag=" + tag,
    __FILE__, __LINE__, __func__);
}
#
void FleetManagerNode::enqueue_robot_action(
  const std::string & robot_id, int priority, const std::string & tag, std::function<void()> fn)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto & pd = pending_dispatch_by_robot_[robot_id];
  // Coalesce: keep the highest priority action (lowest number).
  if (!pd.action_fn.has_value() || priority < pd.action_priority || pd.action_tag == tag) {
    pd.action_fn = std::move(fn);
    pd.action_priority = priority;
    pd.action_tag = tag;
    pd.updated_at = this->now();
    pd.cancel_executed_this_tick = false;
    PersistLogger::log_info(
      "dispatch.enqueue_action", robot_id, "",
      "tag=" + tag + " pri=" + std::to_string(priority),
      __FILE__, __LINE__, __func__);
  } else {
    PersistLogger::log_info(
      "dispatch.enqueue_skip", robot_id, "",
      "skip tag=" + tag + " pri=" + std::to_string(priority) +
        " due to pending tag=" + pd.action_tag + " pri=" + std::to_string(pd.action_priority),
      __FILE__, __LINE__, __func__);
  }
}
#
void FleetManagerNode::request_start_path_navigation(
  const std::string & robot_id,
  const std::string & target_waypoint,
  const std::string & task_id)
{
  enqueue_robot_action(
    robot_id, kPriNav, "nav.start",
    [this, robot_id, target_waypoint, task_id]() {
      (void)this->start_path_navigation(robot_id, target_waypoint, task_id);
    });
}
#
void FleetManagerNode::request_apply_hard_hold(
  const std::string & robot_id,
  const std::string & reason,
  const std::string & wait_for_robot,
  bool release_when_peer_far,
  bool snapshot_resume)
{
  enqueue_robot_action(
    robot_id, kPriHold, "hold.apply",
    [this, robot_id, reason, wait_for_robot, release_when_peer_far, snapshot_resume]() {
      this->apply_hard_hold(robot_id, reason, wait_for_robot, release_when_peer_far, snapshot_resume);
    });
}
#
void FleetManagerNode::request_stage_safe_hold(
  const std::string & robot_id,
  const std::string & hold_waypoint,
  const std::string & reason,
  const std::string & wait_for_robot,
  bool release_when_peer_far,
  const std::string & pending_trigger_avoidance_robot,
  const std::string & pending_trigger_avoidance_from,
  bool snapshot_resume,
  bool force_move_to_hold_waypoint)
{
  enqueue_robot_action(
    robot_id, kPriHold, "hold.stage",
    [this, robot_id, hold_waypoint, reason, wait_for_robot, release_when_peer_far,
     pending_trigger_avoidance_robot, pending_trigger_avoidance_from,
     snapshot_resume, force_move_to_hold_waypoint]() {
      this->stage_safe_hold(robot_id, hold_waypoint, reason, wait_for_robot, release_when_peer_far,
                            pending_trigger_avoidance_robot, pending_trigger_avoidance_from,
                            snapshot_resume, force_move_to_hold_waypoint);
    });
}
#
void FleetManagerNode::request_release_hard_hold(const std::string & robot_id, const std::string & reason)
{
  enqueue_robot_action(
    robot_id, kPriHold, "hold.release",
    [this, robot_id, reason]() { this->release_hard_hold(robot_id, reason); });
}
#
void FleetManagerNode::request_move_idle_robot_to_avoidance(
  const std::string & idle_robot_id,
  const std::string & avoidance_waypoint,
  bool force)
{
  enqueue_robot_action(
    idle_robot_id, kPriAvoidance, "avoidance.move",
    [this, idle_robot_id, avoidance_waypoint, force]() {
      (void)this->move_idle_robot_to_avoidance(idle_robot_id, avoidance_waypoint, force);
    });
}

void FleetManagerNode::request_process_hold_state_machine()
{
  enqueue_robot_action(
    kGlobalHoldKey, kPriGlobal, "global.hold_sm",
    [this]() { this->process_hold_state_machine(); });
}

void FleetManagerNode::request_publish_traffic_fleet_status()
{
  enqueue_robot_action(
    kGlobalPublishKey, kPriGlobal, "global.publish_status",
    [this]() { this->publish_traffic_fleet_status(); });
}
#
void FleetManagerNode::dispatch_timer_callback()
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  if (pending_dispatch_by_robot_.empty()) return;
#
  const auto now = this->now();
  size_t global_settling = 0;
  for (const auto & [rid, nav] : robot_nav_info_) {
    if (!nav) continue;
    const bool in_settling = (nav->recent_cancel_until.nanoseconds() > 0 && now < nav->recent_cancel_until);
    const bool in_commit = (nav->nav_control_commit_until.nanoseconds() > 0 && now < nav->nav_control_commit_until);
    if (in_settling || in_commit) global_settling++;
  }
#
  // Process robots in a stable order to keep behavior deterministic.
  std::vector<std::string> robots;
  robots.reserve(pending_dispatch_by_robot_.size());
  for (const auto & kv : pending_dispatch_by_robot_) robots.push_back(kv.first);
  std::sort(robots.begin(), robots.end());
  std::stable_sort(robots.begin(), robots.end(),
                   [](const std::string & a, const std::string & b) {
                     const bool a_global = a.rfind(kGlobalKey, 0) == 0;
                     const bool b_global = b.rfind(kGlobalKey, 0) == 0;
                     if (a_global && !b_global) return true;
                     if (b_global && !a_global) return false;
                     return a < b;
                   });
#
  size_t processed = 0;
  for (const auto & rid : robots) {
    if (processed >= dispatch_max_robots_per_tick_) break;
    auto it = pending_dispatch_by_robot_.find(rid);
    if (it == pending_dispatch_by_robot_.end()) continue;
    auto & pd = it->second;
#
    // 1) Cancel first. Never do cancel+action for same robot in same tick.
    if (pd.cancel_fn.has_value() && !pd.cancel_executed_this_tick) {
      try {
        (*pd.cancel_fn)();
      } catch (...) {}
      pd.cancel_fn.reset();
      pd.cancel_executed_this_tick = true;
      processed++;
      continue;
    }
#
    if (pd.cancel_executed_this_tick) {
      // Defer action to next tick.
      pd.cancel_executed_this_tick = false;
      continue;
    }
#
    // 2) Execute at most one action. Global gate to reduce action churn.
    if (pd.action_fn.has_value()) {
      if (pd.action_priority >= kPriNav && global_settling >= 2) {
        PersistLogger::log_info(
          "dispatch.global_gate_defer", rid, "",
          "defer action tag=" + pd.action_tag +
            " due to global_settling=" + std::to_string(global_settling),
          __FILE__, __LINE__, __func__);
        continue;
      }
      try {
        (*pd.action_fn)();
      } catch (...) {}
      pd.action_fn.reset();
      pd.action_priority = 100;
      pd.action_tag.clear();
      processed++;
    }
#
    // Cleanup empty entries.
    if (!pd.cancel_fn.has_value() && !pd.action_fn.has_value()) {
      pending_dispatch_by_robot_.erase(it);
    }
  }
}
#
}  // namespace fleet_manager

