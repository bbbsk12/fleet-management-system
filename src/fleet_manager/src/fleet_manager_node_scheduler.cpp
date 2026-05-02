#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#include <tf2/LinearMath/Quaternion.hpp>

namespace fleet_manager
{
// ==================== 任务调度 ====================

void FleetManagerNode::scheduler_timer_callback()
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  // 定期清理超时预约锁（防止异常中断后残留预约永久封锁航点）
  occupancy_manager_->expire_stale_reservations();
  // 定期清理已结束任务（防止内存泄漏）
  task_scheduler_->purge_finished_tasks(200);
  sync_task_scheduler_with_holds();
  recover_orphaned_tasks();
  assign_pending_tasks();
}

void FleetManagerNode::sync_task_scheduler_with_holds()
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  const auto tasks = task_scheduler_->get_all_tasks();
  for (const auto & t : tasks) {
    if (t.task_id.empty() || t.assigned_robot_id.empty()) continue;
    if (t.status != "pending" && t.status != "assigned") continue;
    auto h_it = hold_contexts_.find(t.assigned_robot_id);
    if (h_it == hold_contexts_.end() || !h_it->second.active) continue;
    if (h_it->second.resume_task_id != t.task_id) continue;
    if (task_scheduler_->set_task_waiting_fleet(t.task_id)) {
      auto info = task_scheduler_->get_task_info(t.task_id);
      if (!info.task_id.empty()) task_status_pub_->publish(info);
    }
  }
}

void FleetManagerNode::recover_orphaned_tasks()
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  const auto now = this->now();
  // Hard invariant: one robot must not carry multiple execution-like tasks at once.
  // If this happens due to race/recovery ordering, keep one and requeue the rest.
  {
    const auto tasks = task_scheduler_->get_all_tasks();
    std::map<std::string, std::vector<fleet_msgs::msg::TaskInfo>> active_by_robot;
    auto is_execution_like_status = [](const std::string & st) {
      return st == "assigned" || st == "waiting_fleet" || st == "in_progress" ||
             st == "executing" || st == "navigating";
    };
    for (const auto & t : tasks) {
      if (t.assigned_robot_id.empty()) continue;
      if (!is_execution_like_status(t.status)) continue;
      active_by_robot[t.assigned_robot_id].push_back(t);
    }
    auto status_rank = [](const std::string & st) {
      if (st == "executing") return 5;
      if (st == "in_progress" || st == "navigating") return 4;
      if (st == "waiting_fleet") return 3;
      if (st == "assigned") return 2;
      return 1;
    };
    for (auto & [rid, vec] : active_by_robot) {
      if (vec.size() <= 1) continue;
      std::stable_sort(
        vec.begin(), vec.end(),
        [&](const fleet_msgs::msg::TaskInfo & a, const fleet_msgs::msg::TaskInfo & b) {
          const int ra = status_rank(a.status);
          const int rb = status_rank(b.status);
          if (ra != rb) return ra > rb;
          if (a.created_at.sec != b.created_at.sec) {
            return a.created_at.sec < b.created_at.sec;
          }
          if (a.created_at.nanosec != b.created_at.nanosec) {
            return a.created_at.nanosec < b.created_at.nanosec;
          }
          if (a.priority != b.priority) {
            return a.priority > b.priority;
          }
          return a.task_id < b.task_id;
        });
      const std::string keep_task_id = vec.front().task_id;
      PersistLogger::log_warn(
        "sched.multi_active_recover", rid, keep_task_id,
        "detected " + std::to_string(vec.size()) +
          " execution-like tasks on one robot; keep=" + keep_task_id + ", requeue others",
        __FILE__, __LINE__, __func__);
      for (size_t i = 1; i < vec.size(); ++i) {
        task_scheduler_->mark_task_pending(vec[i].task_id);
        auto info = task_scheduler_->get_task_info(vec[i].task_id);
        if (!info.task_id.empty()) task_status_pub_->publish(info);
      }
    }
  }

  // Clean up stale internal avoidance states that can get stuck at:
  // current_task_id=avoidance_* with goal=0 and no pending rotate/path.
  // These states are outside task_scheduler tracking and can permanently
  // block follow-up user task dispatch if not self-healed.
  for (auto & [rid, nav] : robot_nav_info_) {
    if (!nav) continue;
    if (nav->current_task_id.rfind("avoidance_", 0) != 0) continue;

    // Internal avoidance can also hang with has_active_goal=1 forever
    // (e.g. Nav2 result callback lost / stale goal handle). That blocks hold chains.
    if (nav->has_active_goal || nav->pre_rotate_pending) {
      const rclcpp::Time active_ref =
        (nav->nav_last_activity.nanoseconds() > 0) ? nav->nav_last_activity :
        (nav->last_goal_issue_time.nanoseconds() > 0) ? nav->last_goal_issue_time :
        nav->nav_active_since;
      if (active_ref.nanoseconds() > 0 && nav_action_stuck_timeout_sec_ > 0.0) {
        const double active_stale_sec = (now - active_ref).seconds();
        if (active_stale_sec >= nav_action_stuck_timeout_sec_) {
          PersistLogger::log_warn(
            "avoidance.active_stale_recover", rid, nav->current_task_id,
            "stale internal avoidance with active goal for " + std::to_string(active_stale_sec) +
              "s, force cancel/reset",
            __FILE__, __LINE__, __func__);
          enqueue_robot_cancel(
            rid, "avoidance.active_stale_recover_cancel",
            [this, rid]() {
              auto ni = get_robot_nav_info(rid);
              if (!ni) return;
              cancel_active_robot_goals(ni);
            });
          enqueue_robot_action(
            rid, 5, "avoidance.active_stale_recover_reset",
            [this, rid]() {
              auto ni = get_robot_nav_info(rid);
              if (!ni) return;
              occupancy_manager_->release_reservations(rid);
              reset_through_segment_state(ni);
              ni->pre_rotate_pending = false;
              ni->pending_waypoint_path.clear();
              ni->pending_through_segment_after_rotate.clear();
              ni->current_goal_handle.reset();
              ni->current_pose_goal_handle.reset();
              ni->current_follow_goal_handle.reset();
              ni->has_active_goal = false;
              ni->route_waypoints.clear();
              ni->current_waypoint_index = 0;
              ni->current_task_id.clear();
              ni->nav_active_since = rclcpp::Time{};
              ni->nav_last_activity = rclcpp::Time{};
              ni->last_goal_issue_time = rclcpp::Time{};
              publish_zero_cmd_vel(rid, 2);
            });
        }
      }
      continue;
    }

    if (!nav->pending_waypoint_path.empty() || !nav->pending_through_segment_after_rotate.empty()) continue;

    const rclcpp::Time stale_ref =
      (nav->nav_last_activity.nanoseconds() > 0) ? nav->nav_last_activity :
      (nav->last_goal_issue_time.nanoseconds() > 0) ? nav->last_goal_issue_time :
      nav->nav_active_since;
    if (stale_ref.nanoseconds() <= 0) continue;

    const double stale_sec = (now - stale_ref).seconds();
    const double stale_timeout_sec = std::max(2.5, nav_action_stuck_timeout_sec_ * 0.35);
    if (stale_sec < stale_timeout_sec) continue;

    PersistLogger::log_warn(
      "avoidance.stale_recover", rid, nav->current_task_id,
      "stale internal avoidance (goal=0) for " + std::to_string(stale_sec) +
      "s, force clear residual nav state",
      __FILE__, __LINE__, __func__);

    // If a user-task hold context is still attached, proactively recover it.
    // Relying on later orphan detection can miss this when task status is
    // not in the checked set, leaving dispatch permanently stalled.
    auto h_it = hold_contexts_.find(rid);
    if (h_it != hold_contexts_.end() &&
        !h_it->second.resume_task_id.empty() &&
        !is_internal_task_id(h_it->second.resume_task_id))
    {
      const std::string recover_task_id = h_it->second.resume_task_id;
      PersistLogger::log_warn(
        "sched.stale_hold_recover", rid, recover_task_id,
        "clear stale hold context after avoidance stale recover, mark task pending",
        __FILE__, __LINE__, __func__);
      task_scheduler_->mark_task_pending(recover_task_id);
      auto info = task_scheduler_->get_task_info(recover_task_id);
      if (!info.task_id.empty()) task_status_pub_->publish(info);
      hold_contexts_.erase(h_it);
      hold_release_delayed_since_.erase(rid);
      robot_blocked_by_.erase(rid);
      robot_blocked_since_.erase(rid);
    }

    occupancy_manager_->release_reservations(rid);
    reset_through_segment_state(nav);
    nav->pre_rotate_pending = false;
    nav->pending_waypoint_path.clear();
    nav->pending_through_segment_after_rotate.clear();
    nav->current_goal_handle.reset();
    nav->current_pose_goal_handle.reset();
    nav->current_follow_goal_handle.reset();
    nav->has_active_goal = false;
    nav->route_waypoints.clear();
    nav->current_waypoint_index = 0;
    nav->current_task_id.clear();
    nav->nav_active_since = rclcpp::Time{};
    nav->nav_last_activity = rclcpp::Time{};
    nav->last_goal_issue_time = rclcpp::Time{};
    publish_zero_cmd_vel(rid, 2);
  }

  const auto tasks = task_scheduler_->get_all_tasks();
  for (const auto & t : tasks) {
    if (t.task_id.empty() || t.assigned_robot_id.empty()) continue;
    // Recover assigned tasks that lost their effective hold/nav flow.
    //
    // IMPORTANT:
    // - `waiting_fleet` is a deliberate "keep execution ownership but retry later" state.
    // - Navigation deferrals (e.g. `nav.lookahead_blocked`) intentionally clear nav binding
    //   and rely on short backoff + redispatch, so `waiting_fleet` can look "idle".
    // - If we eagerly recover `waiting_fleet` to `pending`, we destroy ownership stability
    //   and create oscillation: waiting_fleet -> pending -> assign -> defer -> waiting_fleet...
    //
    // Therefore we only recover:
    // - `assigned` tasks (because they should quickly transition to nav/hold), or
    // - `waiting_fleet` tasks that have been idle LONG ENOUGH and are not under retry-backoff.
    if (t.status == "waiting_fleet" || t.status == "assigned") {
      auto nav = get_robot_nav_info(t.assigned_robot_id);
      if (!nav) continue;
      auto h_it = hold_contexts_.find(t.assigned_robot_id);
      const bool hold_flow_alive =
        (h_it != hold_contexts_.end() && h_it->second.resume_task_id == t.task_id &&
         (h_it->second.active || !h_it->second.pending_hold_waypoint.empty()));
      const bool nav_busy =
        nav->has_active_goal || nav->pre_rotate_pending || !nav->pending_waypoint_path.empty() ||
        !nav->pending_through_segment_after_rotate.empty() || !nav->current_task_id.empty();
      // IMPORTANT: avoid "false orphan" recovery immediately after assignment.
      // There is a short window where the scheduler has set status=assigned but
      // the dispatch/nav state hasn't been flipped yet (nav.start enqueued).
      const double assigned_age_sec = (now - t.started_at).seconds();
      const bool recently_assigned = (assigned_age_sec >= 0.0 && assigned_age_sec < 1.0);

      bool dispatch_pending = false;
      {
        auto pd_it = pending_dispatch_by_robot_.find(t.assigned_robot_id);
        if (pd_it != pending_dispatch_by_robot_.end()) {
          dispatch_pending = pd_it->second.action_fn.has_value() ||
                             static_cast<bool>(pd_it->second.cancel_fn);
        }
      }

      const double started_age_sec = (now - t.started_at).seconds();
      const bool old_enough_to_recover_waiting =
        (started_age_sec >= 0.0 && started_age_sec >= 6.0);
      const bool has_retry_backoff =
        (first_hop_retry_not_before_.find(t.task_id) != first_hop_retry_not_before_.end());

      const bool allow_waiting_recover =
        (t.status != "waiting_fleet") || (old_enough_to_recover_waiting && !has_retry_backoff);

      if (!hold_flow_alive && !nav_busy && !dispatch_pending && !recently_assigned &&
          allow_waiting_recover)
      {
        PersistLogger::log_warn(
          "sched.waiting_recover", t.assigned_robot_id, t.task_id,
          "task status=" + t.status + " but hold/nav flow is idle, recovering to pending",
          __FILE__, __LINE__, __func__);
        task_scheduler_->mark_task_pending(t.task_id);
        auto info = task_scheduler_->get_task_info(t.task_id);
        if (!info.task_id.empty()) task_status_pub_->publish(info);
      }
      continue;
    }
    // 只检查 in_progress / navigating 状态的任务（executing 状态说明在等底盘反馈，不算孤儿）
    if (t.status != "in_progress" && t.status != "navigating") continue;
    // executing 状态的任务由底盘超时检查处理，不走孤儿恢复逻辑
    if (t.status == "executing") continue;
    // 检查是否有 hold 恢复此任务。
    // 仅在 hold 仍处于有效流程（active 或 pending_hold_waypoint 非空）时跳过孤儿恢复。
    // 若只是残留的非激活 hold context，会导致任务永久卡死，不能继续跳过。
    auto h_it = hold_contexts_.find(t.assigned_robot_id);
    if (h_it != hold_contexts_.end() &&
        h_it->second.resume_task_id == t.task_id &&
        (h_it->second.active || !h_it->second.pending_hold_waypoint.empty()))
    {
      continue;
    }
    // 检查该机器人的内部导航状态
    auto nav = get_robot_nav_info(t.assigned_robot_id);
    if (!nav) continue;
    // 活跃导航长时间无反馈/无结果：进行自愈回收，避免任务永久卡在 executing/in_progress。
    const rclcpp::Time activity_ref =
      (nav->nav_last_activity.nanoseconds() > 0) ? nav->nav_last_activity : nav->nav_active_since;
    if ((nav->has_active_goal || nav->pre_rotate_pending) &&
        nav_action_stuck_timeout_sec_ > 0.0 &&
        activity_ref.nanoseconds() > 0 &&
        (now - activity_ref).seconds() >= nav_action_stuck_timeout_sec_) {
      PersistLogger::log_warn(
        "nav.stuck_recover", t.assigned_robot_id, t.task_id,
        "active nav timeout (" + std::to_string((now - activity_ref).seconds()) +
        "s >= " + std::to_string(nav_action_stuck_timeout_sec_) +
        "s), force reset nav state and recover task",
        __FILE__, __LINE__, __func__);
      // Enqueue recovery to dispatch to avoid cancel/goal churn from timer threads.
      enqueue_robot_cancel(
        t.assigned_robot_id, "nav.stuck_recover",
        [this, rid=t.assigned_robot_id]() {
          auto ni = get_robot_nav_info(rid);
          if (!ni) return;
          cancel_active_robot_goals(ni);
        });
      enqueue_robot_action(
        t.assigned_robot_id, 5, "nav.stuck_recover_reset",
        [this, rid=t.assigned_robot_id]() {
          auto ni = get_robot_nav_info(rid);
          if (!ni) return;
          occupancy_manager_->release_reservations(rid);
          reset_through_segment_state(ni);
          ni->pre_rotate_pending = false;
          ni->pending_waypoint_path.clear();
          ni->pending_through_segment_after_rotate.clear();
          ni->current_goal_handle.reset();
          ni->current_pose_goal_handle.reset();
          ni->current_follow_goal_handle.reset();
          ni->has_active_goal = false;
          ni->nav_active_since = rclcpp::Time{};
          ni->nav_last_activity = rclcpp::Time{};
          publish_zero_cmd_vel(rid, 2);
        });
    }
    // Absolute watchdog: even with continuous feedback/spin, a goal should not
    // run unbounded without segment/task progress.
    if ((nav->has_active_goal || nav->pre_rotate_pending) &&
        nav_action_absolute_timeout_sec_ > 0.0 &&
        nav->nav_active_since.nanoseconds() > 0 &&
        (now - nav->nav_active_since).seconds() >= nav_action_absolute_timeout_sec_) {
      PersistLogger::log_warn(
        "nav.absolute_timeout_recover", t.assigned_robot_id, t.task_id,
        "active nav absolute timeout (" + std::to_string((now - nav->nav_active_since).seconds()) +
        "s >= " + std::to_string(nav_action_absolute_timeout_sec_) +
        "s), force reset nav state and recover task",
        __FILE__, __LINE__, __func__);
      enqueue_robot_cancel(
        t.assigned_robot_id, "nav.absolute_timeout_recover",
        [this, rid=t.assigned_robot_id]() {
          auto ni = get_robot_nav_info(rid);
          if (!ni) return;
          cancel_active_robot_goals(ni);
        });
      enqueue_robot_action(
        t.assigned_robot_id, 5, "nav.absolute_timeout_recover_reset",
        [this, rid=t.assigned_robot_id]() {
          auto ni = get_robot_nav_info(rid);
          if (!ni) return;
          occupancy_manager_->release_reservations(rid);
          reset_through_segment_state(ni);
          ni->pre_rotate_pending = false;
          ni->pending_waypoint_path.clear();
          ni->pending_through_segment_after_rotate.clear();
          ni->current_goal_handle.reset();
          ni->current_pose_goal_handle.reset();
          ni->current_follow_goal_handle.reset();
          ni->has_active_goal = false;
          ni->nav_active_since = rclcpp::Time{};
          ni->nav_last_activity = rclcpp::Time{};
          publish_zero_cmd_vel(rid, 2);
        });
    }
    if (nav->current_task_id == t.task_id) {
      // Stale task-binding guard:
      // current_task_id may stay latched while robot is completely idle, which blocks
      // reassignment/recovery and can freeze the fleet behind a stale route owner.
      const bool nav_really_idle =
        !nav->has_active_goal &&
        !nav->pre_rotate_pending &&
        nav->pending_waypoint_path.empty() &&
        nav->pending_through_segment_after_rotate.empty();
      const double bind_idle_sec =
        (activity_ref.nanoseconds() > 0) ? (now - activity_ref).seconds() : 1e9;
      const bool has_started_at =
        (t.started_at.sec != 0 || t.started_at.nanosec != 0);
      const double task_age_sec =
        has_started_at ? (now - t.started_at).seconds() : 1e9;
      constexpr double kStaleTaskBindingSec = 3.0;
      constexpr double kTaskAgeGraceSec = 1.5;
      if (!(nav_really_idle && bind_idle_sec >= kStaleTaskBindingSec && task_age_sec >= kTaskAgeGraceSec)) {
        continue;  // 仍在执行中
      }
      PersistLogger::log_warn(
        "sched.stale_task_binding_recover", t.assigned_robot_id, t.task_id,
        "current_task_id latched but nav idle for " + std::to_string(bind_idle_sec) +
          "s, recovering task to pending",
        __FILE__, __LINE__, __func__);
    }
    if (nav->has_active_goal || nav->pre_rotate_pending) continue;  // 有活跃导航
    if (is_robot_hard_held(t.assigned_robot_id)) continue;  // 任务车在硬 hold，nav 可能已清空
    // Unified invariant:
    // If the robot is currently running an internal coordination task (avoidance/return),
    // do NOT recover the user task as orphaned. The user task may temporarily have no
    // active nav goal while the internal task clears topology, and recovering it here
    // can create multi-task situations (user task requeued + new task assigned).
    if (!nav->current_task_id.empty() && is_internal_task_id(nav->current_task_id)) {
      continue;
    }
    // 机器人空闲但任务仍标记为 in_progress → 孤儿任务
    PersistLogger::log_warn("sched.orphaned_task", t.assigned_robot_id, t.task_id,
      "task status=" + t.status + " but robot nav state is idle (current_task=" +
      nav->current_task_id + "), recovering to pending",
      __FILE__, __LINE__, __func__);
    task_scheduler_->mark_task_pending(t.task_id);
    auto info = task_scheduler_->get_task_info(t.task_id);
    if (!info.task_id.empty()) task_status_pub_->publish(info);
  }
}

void FleetManagerNode::assign_pending_tasks()
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  
  // Do NOT return early just because dispatch_pending_count == 0.
  // We still need to process waiting_fleet redispatch and orphaned tasks.
  
  const auto now = this->now();
  auto status_rank = [](const std::string & st) {
    if (st == "executing") return 5;
    if (st == "in_progress" || st == "navigating") return 4;
    if (st == "waiting_fleet") return 3;
    if (st == "assigned") return 2;
    return 1;
  };
  auto higher_priority_task = [&](const fleet_msgs::msg::TaskInfo & a,
                                  const fleet_msgs::msg::TaskInfo & b) {
    const int ra = status_rank(a.status);
    const int rb = status_rank(b.status);
    if (ra != rb) return ra > rb;  // executing chain always wins over non-executing
    if (a.created_at.sec != b.created_at.sec) return a.created_at.sec < b.created_at.sec;
    if (a.created_at.nanosec != b.created_at.nanosec) return a.created_at.nanosec < b.created_at.nanosec;
    if (a.priority != b.priority) return a.priority > b.priority;
    return a.task_id < b.task_id;
  };

  // Hard invariant: a robot must not be assigned a new task if it already has an
  // execution-like task in the scheduler, even if its nav state is still idle
  // (e.g. nav.start is queued / globally gated).
  std::set<std::string> busy_robots_by_task;
  std::unordered_map<std::string, std::string> active_user_task_by_robot;
  {
    const auto all = task_scheduler_->get_all_tasks();
    auto is_execution_like_status = [](const std::string & st) {
      return st == "assigned" || st == "waiting_fleet" || st == "in_progress" ||
             st == "executing" || st == "navigating";
    };
    for (const auto & t : all) {
      if (t.assigned_robot_id.empty()) continue;
      if (!is_execution_like_status(t.status)) continue;
      busy_robots_by_task.insert(t.assigned_robot_id);
      if (is_internal_task_id(t.task_id)) continue;
      auto it = active_user_task_by_robot.find(t.assigned_robot_id);
      if (it == active_user_task_by_robot.end()) {
        active_user_task_by_robot.emplace(t.assigned_robot_id, t.task_id);
      } else if (it->second != t.task_id) {
        // Resolve owner with strict user rule:
        // executing/in_progress/navigating > waiting_fleet/assigned, then created_at asc,
        // then priority desc, then task_id desc.
        auto owner_info = task_scheduler_->get_task_info(it->second);
        auto candidate_info = task_scheduler_->get_task_info(t.task_id);
        if (!owner_info.task_id.empty() && !candidate_info.task_id.empty() &&
            higher_priority_task(candidate_info, owner_info)) {
          it->second = candidate_info.task_id;
        }
        PersistLogger::log_warn(
          "sched.multi_active_seen", t.assigned_robot_id, it->second,
          "multiple execution-like user tasks seen during assign gate: owner=" + it->second +
            " candidate=" + t.task_id,
          __FILE__, __LINE__, __func__);
      }
    }
  }
  const bool monitor_stale =
    has_monitor_fleet_ &&
    last_monitor_fleet_update_time_.nanoseconds() > 0 &&
    monitor_fleet_stale_timeout_sec_ > 0.0 &&
    (now - last_monitor_fleet_update_time_).seconds() > monitor_fleet_stale_timeout_sec_;
  if (monitor_stale) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "event=scheduler.monitor_stale_pause state_prev=dispatching state_new=pending "
      "reason=FLEET_MONITOR_STALE age=%.2fs timeout=%.2fs",
      (now - last_monitor_fleet_update_time_).seconds(), monitor_fleet_stale_timeout_sec_);
    return;
  }
  task_scheduler_->repair_pending_queue_if_needed();

  // Visibility: print current top pending candidates by strict ordering (throttled).
  {
    auto all_tasks = task_scheduler_->get_all_tasks();
    std::vector<fleet_msgs::msg::TaskInfo> pending_view;
    pending_view.reserve(all_tasks.size());
    for (const auto & t : all_tasks) {
      if (t.status == "pending") {
        pending_view.push_back(t);
      }
    }
    if (!pending_view.empty()) {
      std::stable_sort(
        pending_view.begin(), pending_view.end(),
        [&](const fleet_msgs::msg::TaskInfo & a, const fleet_msgs::msg::TaskInfo & b) {
          // For pending queue, status rank is equal; still reuse unified comparator.
          return higher_priority_task(a, b);
        });
      std::ostringstream oss;
      const size_t n = std::min<size_t>(3, pending_view.size());
      for (size_t i = 0; i < n; ++i) {
        const auto & t = pending_view[i];
        if (i > 0) oss << " | ";
        oss << "#" << (i + 1) << ":" << t.task_id
            << " ts=" << t.created_at.sec << "." << t.created_at.nanosec
            << " pri=" << t.priority
            << " st=" << t.status;
      }
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "event=sched.pending_order detail=%s", oss.str().c_str());
    }
  }

  // Redispatch waiting_fleet tasks (ownership-preserving defers):
  // these tasks should keep their robot ownership but still need periodic
  // nav.start retries after backoff windows.
  {
    const auto all_tasks = task_scheduler_->get_all_tasks();
    for (const auto & t : all_tasks) {
      if (t.status != "waiting_fleet") continue;
      if (t.assigned_robot_id.empty()) continue;
      const std::string & rid = t.assigned_robot_id;
      auto nav_info = get_robot_nav_info(rid);
      if (!nav_info) continue;
      if (is_robot_hard_held(rid)) continue;

      // If this task is already actively running on nav side, no redispatch needed.
      if (nav_info->current_task_id == t.task_id &&
          (nav_info->has_active_goal || nav_info->pre_rotate_pending ||
           !nav_info->pending_waypoint_path.empty() ||
           !nav_info->pending_through_segment_after_rotate.empty())) {
        continue;
      }

      // If nav is occupied by another chain (including internal), do not steal.
      if (!nav_info->current_task_id.empty() && nav_info->current_task_id != t.task_id) {
        continue;
      }
      if (nav_info->has_active_goal || nav_info->pre_rotate_pending ||
          !nav_info->pending_waypoint_path.empty() ||
          !nav_info->pending_through_segment_after_rotate.empty()) {
        continue;
      }

      auto retry_it = first_hop_retry_not_before_.find(t.task_id);
      if (retry_it != first_hop_retry_not_before_.end() && now < retry_it->second) {
        continue;
      }

      PersistLogger::log_info(
        "sched.redispatch_waiting", rid, t.task_id,
        "retry nav.start for waiting_fleet task to_wp=" + t.waypoint_id,
        __FILE__, __LINE__, __func__);
      request_start_path_navigation(rid, t.waypoint_id, t.task_id);
      // Mark as busy for this scheduler tick to prevent parallel reassignment.
      busy_robots_by_task.insert(rid);
      active_user_task_by_robot[rid] = t.task_id;
    }
  }

  std::map<std::string, std::string> fixed_pending_task_by_robot;
  for (const auto & task : task_scheduler_->get_all_tasks()) {
    if (task.status == "pending" && !task.assigned_robot_id.empty()) {
      fixed_pending_task_by_robot.emplace(task.assigned_robot_id, task.task_id);
    }
  }

  auto clear_internal_coordination_state =
    [&](const std::string & robot_id,
        const std::shared_ptr<RobotNavigationInfo> & nav_info,
        const std::string & fixed_task_id,
        const std::string & reason) {
      if (!nav_info) {
        return;
      }

      RCLCPP_WARN(this->get_logger(),
                  "event=scheduler.preempt task=%s robot=%s state_prev=busy state_new=idle reason=%s detail=%s",
                  fixed_task_id.c_str(), robot_id.c_str(), reason.c_str(),
                  nav_state_string(robot_id, nav_info).c_str());

      cancel_active_robot_goals(nav_info);
      occupancy_manager_->release_reservations(robot_id);
      hold_contexts_.erase(robot_id);
      hold_release_delayed_since_.erase(robot_id);
      robot_blocked_by_.erase(robot_id);
      robot_blocked_since_.erase(robot_id);
      robot_avoidance_cooldown_until_.erase(robot_id);

      reset_through_segment_state(nav_info);
      nav_info->pre_rotate_pending = false;
      nav_info->pending_waypoint_path.clear();
      nav_info->pending_through_segment_after_rotate.clear();
      nav_info->current_task_id.clear();
      nav_info->route_waypoints.clear();
      nav_info->current_waypoint_index = 0;
      nav_info->has_active_goal = false;
      publish_zero_cmd_vel(robot_id, 3);
    };

  auto can_preempt_for_fixed_pending_task =
    [&](const std::shared_ptr<RobotNavigationInfo> & nav_info) -> bool {
      if (!nav_info) return false;
      // In-flight protection: never preempt while navigation is actively progressing.
      if (nav_info->has_active_goal || nav_info->pre_rotate_pending ||
          !nav_info->pending_waypoint_path.empty() ||
          !nav_info->pending_through_segment_after_rotate.empty()) {
        return false;
      }
      // Keep internal coordination ownership for a short guard window even after
      // active goal drops, to avoid rapid cancel/reassign thrash.
      const auto ref = (nav_info->nav_last_activity.nanoseconds() > 0) ?
        nav_info->nav_last_activity : nav_info->last_goal_issue_time;
      if (ref.nanoseconds() > 0) {
        constexpr double kInFlightPreemptGuardSec = 2.0;
        if ((now - ref).seconds() < kInFlightPreemptGuardSec) {
          return false;
        }
      }
      return true;
    };

  std::vector<fleet_msgs::msg::RobotStatus> online_robots;
  for (const auto & [robot_id, status] : robot_statuses_) {
    auto nav_info = get_robot_nav_info(robot_id);
    if (!nav_info) continue;

    const auto fixed_it = fixed_pending_task_by_robot.find(robot_id);
    const bool has_fixed_pending_task = fixed_it != fixed_pending_task_by_robot.end();
    const std::string fixed_task_id = has_fixed_pending_task ? fixed_it->second : "";

    // 信任 fleet_monitor 的连接状态判定
    if (status.connection_status != "online") {
      // 备选：action server 已就绪也视为可用（fleet_monitor 判定延迟时的兜底）
      const bool action_ready =
        (nav_info->nav_client && nav_info->nav_client->action_server_is_ready()) ||
        (nav_info->nav_through_client && nav_info->nav_through_client->action_server_is_ready()) ||
        (nav_info->follow_waypoints_client && nav_info->follow_waypoints_client->action_server_is_ready());
      if (!action_ready) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Robot %s skipped: connection_status=%s, action_servers not ready",
          robot_id.c_str(), status.connection_status.c_str());
        if (has_fixed_pending_task) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "event=task.defer task=%s robot=%s state_prev=pending state_new=pending reason=ROBOT_UNAVAILABLE detail=connection_status:%s action_server:not_ready",
            fixed_task_id.c_str(), robot_id.c_str(), status.connection_status.c_str());
        }
        continue;
      }
    }

    // 定位置信度检查：位姿全零说明无定位，不应接收新任务
    {
      const auto & p = status.current_pose.position;
      if (std::abs(p.x) < 1e-6 && std::abs(p.y) < 1e-6) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Robot %s skipped: no valid pose (0,0)", robot_id.c_str());
        continue;
      }
    }

    if (is_robot_hard_held(robot_id)) {
      const auto hold_it = hold_contexts_.find(robot_id);
      const bool internal_hold =
        hold_it != hold_contexts_.end() &&
        (hold_it->second.resume_task_id.empty() || is_internal_task_id(hold_it->second.resume_task_id));
      if (has_fixed_pending_task && internal_hold) {
        if (!can_preempt_for_fixed_pending_task(nav_info)) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "event=task.defer task=%s robot=%s state_prev=pending state_new=pending reason=IN_FLIGHT_PROTECTION",
            fixed_task_id.c_str(), robot_id.c_str());
          continue;
        }
        clear_internal_coordination_state(robot_id, nav_info, fixed_task_id, "hard_hold");
      } else {
        if (has_fixed_pending_task) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "event=task.defer task=%s robot=%s state_prev=pending state_new=waiting_fleet reason=HARD_HOLD_ACTIVE detail=%s",
            fixed_task_id.c_str(), robot_id.c_str(), nav_state_string(robot_id, nav_info).c_str());
        }
        continue;
      }
    }

    // 跳过处于 pending hold（退避中、等待到位）的机器人
    {
      auto ph_it = hold_contexts_.find(robot_id);
      if (ph_it != hold_contexts_.end() && !ph_it->second.pending_hold_waypoint.empty()) {
        continue;
      }
    }

    // 跳过正在等底盘执行反馈的机器人（executing 状态）
    if (is_robot_executing(robot_id)) {
      continue;
    }

    // Skip robots that already carry an execution-like task in the scheduler.
    // This closes the gap where status=assigned but nav state hasn't been marked busy yet.
    if (busy_robots_by_task.count(robot_id)) {
      continue;
    }

    // Avoid assigning new user tasks onto robots that are in a coordination/avoidance
    // cooldown window. These robots were recently asked to clear traffic as blockers;
    // letting them immediately pick up new user tasks can create priority inversions
    // (later task reserves the same chokepoint that earlier executing tasks need).
    {
      auto cd_it = robot_avoidance_cooldown_until_.find(robot_id);
      if (cd_it != robot_avoidance_cooldown_until_.end() && now < cd_it->second) {
        // Still allow fixed pending tasks (explicitly bound) to be considered; those
        // are handled via fixed-task preemption logic later in this function.
        if (!has_fixed_pending_task) {
          continue;
        }
      }
    }

    const bool busy_nav_state =
      nav_info->pre_rotate_pending || !nav_info->pending_waypoint_path.empty() ||
      nav_info->has_active_goal || !nav_info->current_task_id.empty() ||
      !nav_info->route_waypoints.empty();
    if (busy_nav_state) {
      const bool internal_or_residual_state =
        nav_info->current_task_id.empty() || is_internal_task_id(nav_info->current_task_id);
      if (has_fixed_pending_task && internal_or_residual_state) {
        // Round 10b: 如果有其它机器人处于 hard_hold 且正在等待本机器人让行，
        // 本机当前的内部任务（避让）是 hold 协调的一部分，不能被用户任务覆盖
        bool needed_for_hold_resolution = false;
        if (is_internal_task_id(nav_info->current_task_id)) {
          for (const auto & [hid, hctx] : hold_contexts_) {
            if (hctx.active && hctx.wait_for_robot == robot_id) {
              needed_for_hold_resolution = true;
              break;
            }
          }
        }
        if (needed_for_hold_resolution) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "event=task.defer task=%s robot=%s state_prev=pending state_new=pending reason=HOLD_RESOLUTION_IN_PROGRESS",
            fixed_task_id.c_str(), robot_id.c_str());
          continue;
        }
        if (!can_preempt_for_fixed_pending_task(nav_info)) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "event=task.defer task=%s robot=%s state_prev=pending state_new=pending reason=IN_FLIGHT_PROTECTION",
            fixed_task_id.c_str(), robot_id.c_str());
          continue;
        }
        clear_internal_coordination_state(robot_id, nav_info, fixed_task_id, "internal_or_residual_nav_state");
      } else {
        if (has_fixed_pending_task) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "event=task.defer task=%s robot=%s state_prev=pending state_new=pending reason=ROBOT_BUSY detail=%s",
            fixed_task_id.c_str(), robot_id.c_str(), nav_state_string(robot_id, nav_info).c_str());
        }
        continue;
      }
    }

    auto status_copy = status;
    status_copy.connection_status = "online";
    online_robots.push_back(status_copy);
  }

  if (online_robots.empty()) {
    // 有 pending 任务但没有可用机器人时，周期性提示
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "event=scheduler.no_capacity task=- robot=- state_prev=pending state_new=pending reason=NO_AVAILABLE_ROBOTS detail=pending_count:%zu known_robots:%zu",
      task_scheduler_->get_dispatch_pending_count(), robot_statuses_.size());
    return;
  }

  auto waypoint_poses = traffic_manager_->get_all_waypoint_poses();

  // 批量分配：一次性将所有 pending 任务分配给所有可用机器人
  auto assigned_tasks = task_scheduler_->assign_tasks_batch(online_robots, waypoint_poses);

  for (auto & assigned_task : assigned_tasks) {
    if (assigned_task.task_id.empty()) continue;
    // Final safety net: even if a robot slipped through pre-filtering (race between hold/nav
    // state and scheduler view), never let one robot accumulate multiple user tasks.
    {
      const std::string & rid = assigned_task.assigned_robot_id;
      if (!rid.empty()) {
        // If robot is hard-held for a DIFFERENT user task, requeue this task.
        if (is_robot_hard_held(rid)) {
          auto h_it = hold_contexts_.find(rid);
          const bool holding_other_user_task =
            (h_it != hold_contexts_.end() &&
             !h_it->second.resume_task_id.empty() &&
             !is_internal_task_id(h_it->second.resume_task_id) &&
             h_it->second.resume_task_id != assigned_task.task_id);
          if (holding_other_user_task) {
            PersistLogger::log_warn(
              "sched.assign_skip_held", rid, assigned_task.task_id,
              "robot hard-held for other task=" + h_it->second.resume_task_id + ", requeue",
              __FILE__, __LINE__, __func__);
            task_scheduler_->mark_task_pending(assigned_task.task_id);
            continue;
          }
        }
        auto nav_info = get_robot_nav_info(rid);
        auto owner_it = active_user_task_by_robot.find(rid);
        if (owner_it != active_user_task_by_robot.end() &&
            owner_it->second != assigned_task.task_id) {
          PersistLogger::log_warn(
            "sched.assign_skip_active_owner", rid, assigned_task.task_id,
            "robot already owns execution-like task=" + owner_it->second + ", requeue",
            __FILE__, __LINE__, __func__);
          task_scheduler_->mark_task_pending(assigned_task.task_id);
          continue;
        }
        // If nav is currently executing an internal task (avoidance/return), do NOT dispatch
        // a new user task onto this robot; it causes user-task interruption thrash.
        if (nav_info &&
            !nav_info->current_task_id.empty() &&
            is_internal_task_id(nav_info->current_task_id) &&
            !is_internal_task_id(assigned_task.task_id)) {
          PersistLogger::log_warn(
            "sched.assign_skip_internal_nav", rid, assigned_task.task_id,
            "robot nav executing internal task=" + nav_info->current_task_id + ", requeue",
            __FILE__, __LINE__, __func__);
          task_scheduler_->mark_task_pending(assigned_task.task_id);
          continue;
        }
        // If nav is already bound to a different user task, requeue.
        if (nav_info &&
            !nav_info->current_task_id.empty() &&
            !is_internal_task_id(nav_info->current_task_id) &&
            nav_info->current_task_id != assigned_task.task_id) {
          PersistLogger::log_warn(
            "sched.assign_skip_busy_nav", rid, assigned_task.task_id,
            "robot nav bound to other task=" + nav_info->current_task_id + ", requeue",
            __FILE__, __LINE__, __func__);
          task_scheduler_->mark_task_pending(assigned_task.task_id);
          continue;
        }
      }
    }
    const auto retry_it = first_hop_retry_not_before_.find(assigned_task.task_id);
    if (retry_it != first_hop_retry_not_before_.end() && this->now() < retry_it->second) {
      // 退避窗口内不重复派发，避免 assign/defer 高频抖动。
      task_scheduler_->mark_task_pending(assigned_task.task_id);
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "event=sched.backoff_skip task=%s robot=%s state_prev=pending state_new=pending "
        "reason=FIRST_HOP_BACKOFF_WINDOW detail=not_before_ns:%ld",
        assigned_task.task_id.c_str(), assigned_task.assigned_robot_id.c_str(),
        retry_it->second.nanoseconds());
      continue;
    }
    PersistLogger::log_info(
      "sched.assign", assigned_task.assigned_robot_id, assigned_task.task_id,
      "assigned to_wp=" + assigned_task.waypoint_id,
      __FILE__, __LINE__, __func__);
    // Queue the side-effect; dispatch timer will serialize nav goal emission.
    request_start_path_navigation(
      assigned_task.assigned_robot_id, assigned_task.waypoint_id, assigned_task.task_id);
  }
}

}  // namespace fleet_manager
