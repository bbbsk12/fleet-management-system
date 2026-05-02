#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#include <tf2/LinearMath/Quaternion.hpp>

namespace fleet_manager
{
// ==================== Hold 管理 ====================

bool FleetManagerNode::should_trigger_blocker_avoidance(
  const std::string & requester_robot_id,
  const std::string & blocker_robot_id,
  const std::string & target_waypoint,
  const std::string & task_id,
  const std::string & context_tag)
{
  const auto now = this->now();
  const std::string pair_key =
    (requester_robot_id < blocker_robot_id)
    ? (requester_robot_id + "|" + blocker_robot_id)
    : (blocker_robot_id + "|" + requester_robot_id);

  std::string blocker_signature = "nav_missing";
  auto blocker_nav_it = robot_nav_info_.find(blocker_robot_id);
  if (blocker_nav_it != robot_nav_info_.end() && blocker_nav_it->second) {
    const auto & blocker_nav = blocker_nav_it->second;
    const bool blocker_busy =
      blocker_nav->has_active_goal ||
      blocker_nav->pre_rotate_pending ||
      !blocker_nav->pending_waypoint_path.empty();
    blocker_signature = blocker_nav->current_task_id + "|" + (blocker_busy ? "busy" : "idle");

    // In-flight dedup: if blocker is already running its own avoidance task, don't re-trigger.
    if (blocker_nav->current_task_id == ("avoidance_" + blocker_robot_id) && blocker_busy) {
      PersistLogger::log_info(
        "hold.trigger_blocker_avoidance_skip", requester_robot_id, task_id,
        "skip re-trigger for blocker " + blocker_robot_id +
          ": already in-flight " + blocker_nav->current_task_id +
          " context=" + context_tag,
        __FILE__, __LINE__, __func__);
      return false;
    }
  }

  auto target_it = hold_blocker_avoidance_last_target_.find(pair_key);
  auto sig_it = hold_blocker_avoidance_last_signature_.find(pair_key);
  auto cooldown_it = hold_blocker_avoidance_not_before_.find(pair_key);

  const bool same_target = (target_it != hold_blocker_avoidance_last_target_.end() &&
                            target_it->second == target_waypoint);
  const bool same_signature = (sig_it != hold_blocker_avoidance_last_signature_.end() &&
                               sig_it->second == blocker_signature);
  const bool in_cooldown = (cooldown_it != hold_blocker_avoidance_not_before_.end() &&
                            now < cooldown_it->second);

  if (same_target && same_signature && in_cooldown) {
    PersistLogger::log_info(
      "hold.trigger_blocker_avoidance_skip", requester_robot_id, task_id,
      "skip re-trigger for blocker " + blocker_robot_id +
        ": cooldown active target=" + target_waypoint +
        " context=" + context_tag,
      __FILE__, __LINE__, __func__);
    return false;
  }

  // Important: do NOT permanently suppress re-trigger when state is unchanged.
  // In long topology-clear defer loops, blocker/target can remain unchanged for a long
  // time while previous avoidance attempts have already ended/failed. We only suppress
  // within cooldown window; once cooldown expires, allow another nudge attempt.
  if (same_target && same_signature && !in_cooldown) {
    PersistLogger::log_info(
      "hold.trigger_blocker_avoidance_retry", requester_robot_id, task_id,
      "retry trigger for blocker " + blocker_robot_id +
        " after cooldown target=" + target_waypoint +
        " context=" + context_tag,
      __FILE__, __LINE__, __func__);
  }

  hold_blocker_avoidance_last_target_[pair_key] = target_waypoint;
  hold_blocker_avoidance_last_signature_[pair_key] = blocker_signature;
  hold_blocker_avoidance_not_before_[pair_key] =
    now + rclcpp::Duration::from_seconds(hold_blocker_avoidance_cooldown_sec_);
  return true;
}

void FleetManagerNode::invalidate_hold_blocker_avoidance_pair(
  const std::string & requester_robot_id,
  const std::string & blocker_robot_id)
{
  const std::string pair_key =
    (requester_robot_id < blocker_robot_id)
    ? (requester_robot_id + "|" + blocker_robot_id)
    : (blocker_robot_id + "|" + requester_robot_id);
  hold_blocker_avoidance_last_target_.erase(pair_key);
  hold_blocker_avoidance_last_signature_.erase(pair_key);
  hold_blocker_avoidance_not_before_.erase(pair_key);
}

std::string FleetManagerNode::choose_blocker_avoidance_target(
  const std::string & anchor_wp,
  const std::vector<std::string> & requester_route,
  const std::string & blocker_robot_id) const
{
  if (anchor_wp.empty()) return "";
  std::vector<std::string> exclude = requester_route;
  std::string target = occupancy_manager_->find_avoidance_waypoint(anchor_wp, exclude);
  if (target.empty()) target = anchor_wp;

  // Core fix: blocker target must not be on requester route.
  if (std::find(requester_route.begin(), requester_route.end(), target) != requester_route.end()) {
    exclude.push_back(target);
    const std::string alt = occupancy_manager_->find_avoidance_waypoint(anchor_wp, exclude);
    if (!alt.empty()) target = alt;
  }

  // Avoid noop assignment.
  auto st_it = robot_statuses_.find(blocker_robot_id);
  if (st_it != robot_statuses_.end() &&
      !st_it->second.current_waypoint.empty() &&
      st_it->second.current_waypoint == target)
  {
    exclude.push_back(target);
    const std::string alt = occupancy_manager_->find_avoidance_waypoint(anchor_wp, exclude);
    if (!alt.empty()) target = alt;
  }

  if (target.empty()) return "";
  if (std::find(requester_route.begin(), requester_route.end(), target) != requester_route.end()) {
    return "";
  }
  return target;
}

void FleetManagerNode::apply_hard_hold(
  const std::string & robot_id,
  const std::string & reason,
  const std::string & wait_for_robot,
  bool release_when_peer_far,
  bool snapshot_resume)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info) return;

  // 行为约束：任务车/阻塞等待类 hold 不应停在航段上。
  // 若当前离散位置在 segment，则先“滚到最近航点”再进入 hold（stage_safe_hold），
  // 避免在航线上停车造成后续死锁/拥堵。
  {
    auto st_it = robot_statuses_.find(robot_id);
    const bool is_segment = (st_it != robot_statuses_.end() && st_it->second.location_type == "segment");
    const bool is_blocking_wait =
      (reason.find("blocking") != std::string::npos ||
       reason.find("dead_end") != std::string::npos ||
       reason.find("next_goal_wait_peer_clear") != std::string::npos ||
       reason.find("avoidance_mutual_dead_end_retreat") != std::string::npos);
    // Even if the discrete location was classified as "waypoint", in practice the robot can still
    // be inside the acceptance radius but physically on the lane. For blocking/force-wait holds,
    // use a stricter proximity check to decide whether we must "roll to waypoint" first.
    bool treat_as_segment_for_hold = is_segment;
    std::string nearest_wp_for_hold;
    if (!is_segment && is_blocking_wait && st_it != robot_statuses_.end()) {
      nearest_wp_for_hold = st_it->second.current_waypoint;
      if (nearest_wp_for_hold.empty()) {
        nearest_wp_for_hold = traffic_manager_->find_nearest_waypoint(st_it->second.current_pose, 5.0);
      }
      if (!nearest_wp_for_hold.empty()) {
        const auto wp_pose = traffic_manager_->get_waypoint_pose(nearest_wp_for_hold);
        const double dx = st_it->second.current_pose.position.x - wp_pose.position.x;
        const double dy = st_it->second.current_pose.position.y - wp_pose.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > std::max(0.08, waypoint_acceptance_radius_ * 0.50)) {
          treat_as_segment_for_hold = true;
        }
      }
    }
    if (treat_as_segment_for_hold && is_blocking_wait &&
        !nav_info->current_task_id.empty() &&
        is_internal_task_id(nav_info->current_task_id))
    {
      // Internal avoidance while on segment should keep clearing the lane,
      // not convert into a segment hold that can freeze traffic.
      PersistLogger::log_info(
        "hold.segment_internal_avoidance_skip", robot_id, nav_info->current_task_id,
        "skip segment hold for internal task reason=" + reason +
          " wait_for=" + wait_for_robot,
        __FILE__, __LINE__, __func__);
      return;
    }
    if (treat_as_segment_for_hold && is_blocking_wait && !wait_for_robot.empty()) {
      std::string nearest_wp = traffic_manager_->find_nearest_waypoint(st_it->second.current_pose, 5.0);
      if (!nearest_wp.empty()) {
        // 如果实际上已经很接近某个航点（在 acceptance radius 内），允许直接 hold，
        // 避免 staged-hold promote 反过来再次触发 stage_from_segment 造成自激循环。
        const auto wp_pose = traffic_manager_->get_waypoint_pose(nearest_wp);
        const double dx = st_it->second.current_pose.position.x - wp_pose.position.x;
        const double dy = st_it->second.current_pose.position.y - wp_pose.position.y;
        const bool already_at_waypoint = (std::sqrt(dx * dx + dy * dy) <= waypoint_acceptance_radius_);
        if (already_at_waypoint) {
          // fallthrough to normal apply_hard_hold
        } else {
          // 若已经在同一目标点上有 staged hold，则不重复 stage
          auto h_it = hold_contexts_.find(robot_id);
          if (h_it != hold_contexts_.end() && !h_it->second.active &&
              h_it->second.pending_hold_waypoint == nearest_wp) {
            PersistLogger::log_info(
              "hold.stage_from_segment_skip", robot_id, nav_info->current_task_id,
              "skip duplicate stage to " + nearest_wp + " reason=" + reason +
                " wait_for=" + wait_for_robot,
              __FILE__, __LINE__, __func__);
            // Important: when a staged hold is being promoted at waypoint,
            // don't early-return here, otherwise we may leave an internal
            // avoidance task with goal=0/hold=0 while locks remain.
            // Keep going and promote to active hold below.
          } else {
            PersistLogger::log_info(
              "hold.stage_from_segment", robot_id, nav_info->current_task_id,
              "stage hold from segment to nearest_wp=" + nearest_wp +
                " reason=" + reason + " wait_for=" + wait_for_robot,
              __FILE__, __LINE__, __func__);
            const bool force_move_to_waypoint =
              (!nav_info->current_task_id.empty() && is_internal_task_id(nav_info->current_task_id));
            stage_safe_hold(
              robot_id, nearest_wp, reason, wait_for_robot, release_when_peer_far,
              /*pending_trigger_avoidance_robot=*/"", /*pending_trigger_avoidance_from=*/"",
              snapshot_resume,
              /*force_move_to_hold_waypoint=*/force_move_to_waypoint);
            return;
          }
        }
      }
    }
    if (is_blocking_wait &&
        nav_info->has_active_goal &&
        nav_info->nav_control_commit_until.nanoseconds() > 0 &&
        this->now() < nav_info->nav_control_commit_until)
    {
      PersistLogger::log_info(
        "hold.commit_defer", robot_id, nav_info->current_task_id,
        "defer hold during nav commit window reason=" + reason +
          " wait_for=" + wait_for_robot,
        __FILE__, __LINE__, __func__);
      return;
    }
  }

  cancel_active_robot_goals(nav_info);
  publish_zero_cmd_vel(robot_id, 18);

  // Bug-fix: 释放旧预约，防止幽灵锁阻塞其它机器人
  occupancy_manager_->release_reservations(robot_id);

  auto & ctx = hold_contexts_[robot_id];
  ctx.active = true;
  ctx.reason = reason;
  ctx.wait_for_robot = wait_for_robot;
  ctx.release_when_peer_far = release_when_peer_far;
  ctx.pending_hold_waypoint.clear();

  if (snapshot_resume) {
    if (ctx.resume_route.empty()) {
      ctx.resume_route = nav_info->route_waypoints;
    }
    if (ctx.resume_task_id.empty() && !nav_info->current_task_id.empty()) {
      ctx.resume_task_id = nav_info->current_task_id;
    }
  }

  nav_info->pre_rotate_pending = false;
  nav_info->has_active_goal = false;
  nav_info->pending_waypoint_path.clear();
  nav_info->pending_through_segment_after_rotate.clear();
  reset_through_segment_state(nav_info);

  RCLCPP_WARN(
    this->get_logger(),
    "event=hold.apply task=%s robot=%s state_prev=navigating state_new=hold reason=%s detail=wait_for:%s",
    nav_info->current_task_id.empty() ? "-" : nav_info->current_task_id.c_str(),
    robot_id.c_str(), reason.c_str(), wait_for_robot.empty() ? "-" : wait_for_robot.c_str());
  PersistLogger::log_info(
    "hold", robot_id, nav_info->current_task_id,
    "HOLD applied reason=" + reason + " wait_for=" + wait_for_robot +
    " resume_task=" + ctx.resume_task_id +
    " resume_route_size=" + std::to_string(ctx.resume_route.size()),
    __FILE__, __LINE__, __func__);
}

void FleetManagerNode::stage_safe_hold(
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
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info) return;

  cancel_active_robot_goals(nav_info);
  publish_zero_cmd_vel(robot_id, 10);

  const auto prev_it = hold_contexts_.find(robot_id);
  const bool had_existing = (prev_it != hold_contexts_.end());
  HoldContext prev_ctx;
  if (had_existing) {
    prev_ctx = prev_it->second;
  }
  auto & ctx = hold_contexts_[robot_id];

  ctx.active = false;
  ctx.reason = reason;
  ctx.wait_for_robot = wait_for_robot;
  ctx.pending_hold_waypoint = hold_waypoint;
  ctx.release_when_peer_far = release_when_peer_far;
  ctx.snapshot_resume = snapshot_resume;
  ctx.pending_trigger_avoidance_robot = pending_trigger_avoidance_robot;
  ctx.pending_trigger_avoidance_from = pending_trigger_avoidance_from;

  if (snapshot_resume) {
    if (ctx.resume_route.empty()) {
      ctx.resume_route = nav_info->route_waypoints;
    }
    if (ctx.resume_task_id.empty() && !nav_info->current_task_id.empty()) {
      ctx.resume_task_id = nav_info->current_task_id;
    }
  }

  nav_info->pre_rotate_pending = false;
  nav_info->has_active_goal = false;
  nav_info->pending_waypoint_path.clear();
  nav_info->pending_through_segment_after_rotate.clear();
  reset_through_segment_state(nav_info);

  const bool started = move_idle_robot_to_avoidance(robot_id, hold_waypoint, force_move_to_hold_waypoint);
  if (!started) {
    // Critical: don't overwrite an active hard-hold with a staged hold if we fail
    // to dispatch the "roll to safe waypoint" motion. Otherwise the robot looks unheld
    // but still has a waiting task, causing indefinite traffic_wait.
    if (had_existing) hold_contexts_[robot_id] = prev_ctx;
    else hold_contexts_.erase(robot_id);
  }
}

void FleetManagerNode::release_hard_hold(const std::string & robot_id, const std::string & reason)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto it = hold_contexts_.find(robot_id);
  if (it == hold_contexts_.end()) return;
  auto & ctx = it->second;
  auto nav_info = get_robot_nav_info(robot_id);
  if (!nav_info) {
    hold_contexts_.erase(it);
    return;
  }

  if (!ctx.resume_route.empty() && !ctx.resume_task_id.empty()) {
    const auto saved_task = ctx.resume_task_id;
    const auto saved_route = ctx.resume_route;
    // 获取原目标航点（路线末端）
    const std::string destination = saved_route.back();

    // 重新规划路径而非使用旧快照——hold 期间地图或占用可能已变化
    std::string current_wp;
    auto robot_st = robot_statuses_.find(robot_id);
    if (robot_st != robot_statuses_.end()) {
      current_wp = robot_st->second.current_waypoint;
      if (current_wp.empty()) {
        current_wp = traffic_manager_->find_nearest_waypoint(robot_st->second.current_pose, 2.5);
      }
    }

    std::vector<std::string> new_route;
    if (!current_wp.empty() && current_wp != destination) {
      new_route = traffic_manager_->find_path(current_wp, destination);
    }
    // 如果重新规划失败则回退到旧路线
    if (new_route.empty()) {
      new_route = saved_route;
    }
    new_route = dedup_consecutive_waypoints(new_route);
    if (new_route.empty()) {
      new_route.push_back(destination);
    }

    // 保护：topology_clear 触发释放时，先验证恢复路线的前两跳是否可进入。
    // 否则会出现“刚释放就驶离航点，然后立即再次被拦停”的抖动。
    if (reason == "topology_clear" && new_route.size() >= 2) {
      bool can_release = true;
      const std::string blocker1 =
        occupancy_manager_->check_can_enter(robot_id, new_route[0], new_route[1]);
      std::string blocker2;
      if (!blocker1.empty()) {
        can_release = false;
      } else if (new_route.size() >= 3) {
        blocker2 =
          occupancy_manager_->check_can_enter(robot_id, new_route[1], new_route[2]);
        if (!blocker2.empty()) {
          can_release = false;
        }
      }
      if (!can_release) {
        const auto now = this->now();
        auto delayed_it = hold_release_delayed_since_.find(robot_id);
        if (delayed_it == hold_release_delayed_since_.end() ||
            delayed_it->second.nanoseconds() <= 0) {
          hold_release_delayed_since_[robot_id] = now;
          delayed_it = hold_release_delayed_since_.find(robot_id);
        }
        const double defer_sec = (now - delayed_it->second).seconds();
        const std::string blocker = !blocker1.empty() ? blocker1 : blocker2;

        // If the actual blocker differs from the stored wait_for, update the wait target.
        // This avoids waiting on the wrong peer when the blockage shifted to a different robot
        // (e.g. first-hop blocker moved away but second-hop still blocked by another robot).
        if (!blocker.empty() && blocker != ctx.wait_for_robot) {
          PersistLogger::log_info(
            "hold.wait_for_switch", robot_id, saved_task,
            "switch wait_for " + (ctx.wait_for_robot.empty() ? std::string("-") : ctx.wait_for_robot) +
              " -> " + blocker + " due to topology_clear defer (path=" + join_waypoints(new_route) + ")",
            __FILE__, __LINE__, __func__);
          ctx.wait_for_robot = blocker;
        }

        // Self-heal: if deferred for a while and the blocker is exactly the waited peer
        // that is now idle/unheld, clear potential ghost locks/reservations and retry later.
        if (defer_sec >= 4.0 && !blocker.empty()) {
          auto blocker_nav = get_robot_nav_info(blocker);
          const bool blocker_idle =
            !blocker_nav ||
            (!blocker_nav->has_active_goal &&
             !blocker_nav->pre_rotate_pending &&
             blocker_nav->current_task_id.empty());
          const bool blocker_held = is_robot_hard_held(blocker);
          if (blocker_idle && !blocker_held) {
            occupancy_manager_->release_locks(blocker);
            occupancy_manager_->release_reservations(blocker);
            PersistLogger::log_warn(
              "hold.release_self_heal", robot_id, saved_task,
              "topology_clear deferred " + std::to_string(defer_sec) +
                "s, clear ghost occupancy of blocker=" + blocker,
              __FILE__, __LINE__, __func__);
          }
        }

        // If the blocker is yieldable, try nudging it away via avoidance to unblock the resume path.
        if (defer_sec >= 1.0 && !blocker.empty() && robot_yieldable_as_idle_blocker(blocker)) {
          std::string avoid_anchor = new_route.size() >= 3 ? new_route[2] : new_route[1];
          std::string avoid_wp =
            choose_blocker_avoidance_target(avoid_anchor, new_route, blocker);
          if (!avoid_wp.empty()) {
            if (should_trigger_blocker_avoidance(
                  robot_id, blocker, avoid_wp, saved_task, "release_hard_hold"))
            {
              PersistLogger::log_info(
                "hold.trigger_blocker_avoidance", robot_id, saved_task,
                "triggering blocker " + blocker + " to avoidance " + avoid_wp +
                  " (anchor=" + avoid_anchor + ", deferred=" + std::to_string(defer_sec) + "s)",
                __FILE__, __LINE__, __func__);
              request_move_idle_robot_to_avoidance(blocker, avoid_wp, /*force=*/false);
            }
          }
        }

        PersistLogger::log_info(
          "hold.release_defer", robot_id, saved_task,
          "defer release reason=topology_clear: route prefix not clear path=" +
            join_waypoints(new_route) +
            " blocker1=" + blocker1 +
            " blocker2=" + blocker2 +
            " wait_for=" + ctx.wait_for_robot +
            " deferred=" + std::to_string(defer_sec) + "s",
          __FILE__, __LINE__, __func__);
        return;
      }
    }

    nav_info->route_waypoints = new_route;
    nav_info->current_waypoint_index = 0;
    nav_info->current_task_id = saved_task;
    RCLCPP_INFO(
      this->get_logger(),
      "event=hold.release task=%s robot=%s state_prev=hold state_new=in_progress reason=%s detail=resume_route_size:%zu",
      saved_task.c_str(), robot_id.c_str(), reason.c_str(), new_route.size());
    PersistLogger::log_info(
      "hold", robot_id, saved_task,
      "HOLD released reason=" + reason + " re-planned_route=" + join_waypoints(new_route) +
      " (old_route_size=" + std::to_string(saved_route.size()) + ")",
      __FILE__, __LINE__, __func__);
    // Bug-fix: 先 erase hold context 再 navigate，防止 navigate 内部 re-hold 被后续 erase 误删
    occupancy_manager_->release_reservations(robot_id);
    hold_contexts_.erase(it);
    navigate_to_next_waypoint(robot_id);
    if (!saved_task.empty()) {
      task_scheduler_->mark_task_navigating(saved_task);
    }
  } else {
    hold_release_delayed_since_.erase(robot_id);
    nav_info->route_waypoints.clear();
    nav_info->current_waypoint_index = 0;
    nav_info->current_task_id.clear();
    nav_info->has_active_goal = false;
    nav_info->pending_waypoint_path.clear();
    nav_info->pre_rotate_pending = false;
    RCLCPP_INFO(
      this->get_logger(),
      "event=hold.release task=- robot=%s state_prev=hold state_new=idle reason=%s detail=no_resume_route",
      robot_id.c_str(), reason.c_str());
    occupancy_manager_->release_reservations(robot_id);
    hold_contexts_.erase(it);
  }
}

bool FleetManagerNode::is_robot_hard_held(const std::string & robot_id) const
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto it = hold_contexts_.find(robot_id);
  return it != hold_contexts_.end() && it->second.active;
}

void FleetManagerNode::process_hold_state_machine()
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto reached_waypoint = [&](const std::string & /*robot_id*/,
                              const fleet_msgs::msg::RobotStatus & st,
                              const std::string & waypoint_id) -> bool {
    if (waypoint_id.empty()) return false;
    if (st.current_waypoint == waypoint_id) return true;
    const auto wp_pose = traffic_manager_->get_waypoint_pose(waypoint_id);
    const double dx = st.current_pose.position.x - wp_pose.position.x;
    const double dy = st.current_pose.position.y - wp_pose.position.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (st.location_type == "segment") {
      // Segment/waypoint boundary can jitter; use stricter threshold
      // to avoid stage->promote oscillation while still on the lane.
      return dist <= std::max(0.05, waypoint_acceptance_radius_ * 0.50);
    }
    return dist <= waypoint_acceptance_radius_;
  };

  // 1) Promote pending hold
  for (auto & [robot_id, ctx] : hold_contexts_) {
    if (ctx.active || ctx.pending_hold_waypoint.empty()) continue;
    auto st_it = robot_statuses_.find(robot_id);
    if (st_it == robot_statuses_.end()) continue;
    if (reached_waypoint(robot_id, st_it->second, ctx.pending_hold_waypoint)) {
      apply_hard_hold(robot_id,
                      ctx.reason.empty() ? "arrived_avoidance" : ctx.reason,
                      ctx.wait_for_robot,
                      ctx.release_when_peer_far,
                      ctx.snapshot_resume);
      // 死角退避：申请车已到达退让点，触发阻塞者避让
      if (!ctx.pending_trigger_avoidance_robot.empty()) {
        const std::string trigger_robot = ctx.pending_trigger_avoidance_robot;
        const std::string trigger_from = ctx.pending_trigger_avoidance_from;
        ctx.pending_trigger_avoidance_robot.clear();
        ctx.pending_trigger_avoidance_from.clear();
        if (!trigger_from.empty()) {
          std::string trigger_target = choose_blocker_avoidance_target(
            trigger_from, ctx.resume_route, trigger_robot);
          if (trigger_target.empty()) {
            continue;
          }
          if (should_trigger_blocker_avoidance(
                robot_id, trigger_robot, trigger_target, ctx.resume_task_id, "promote_pending_hold"))
          {
            PersistLogger::log_info(
              "hold.trigger_blocker_avoidance", robot_id, ctx.resume_task_id,
              "triggering " + trigger_robot + " to avoidance " + trigger_target +
              " (from=" + trigger_from + ")",
              __FILE__, __LINE__, __func__);
            const bool force_trigger = is_robot_hard_held(trigger_robot);
            if (force_trigger) {
              PersistLogger::log_warn(
                "hold.trigger_blocker_avoidance_force", robot_id, ctx.resume_task_id,
                "trigger robot " + trigger_robot +
                " is hard-held, forcing avoidance to " + trigger_target,
                __FILE__, __LINE__, __func__);
            }
            request_move_idle_robot_to_avoidance(trigger_robot, trigger_target, force_trigger);
          }
        }
      }
    }
  }

  // 2) Release conditions
  std::vector<std::pair<std::string, std::string>> to_release;
  for (auto & [robot_id, ctx] : hold_contexts_) {
    if (!ctx.active) continue;
    if (ctx.wait_for_robot.empty()) continue;

    // 距离释放
    if (ctx.release_when_peer_far) {
      auto self_st = robot_statuses_.find(robot_id);
      auto peer_st = robot_statuses_.find(ctx.wait_for_robot);
      if (self_st != robot_statuses_.end() && peer_st != robot_statuses_.end()) {
        const double dx = self_st->second.current_pose.position.x - peer_st->second.current_pose.position.x;
        const double dy = self_st->second.current_pose.position.y - peer_st->second.current_pose.position.y;
        if (std::sqrt(dx * dx + dy * dy) >= hold_release_distance_) {
          to_release.push_back({robot_id, "peer_far"});
          continue;
        }
      }
    }

    // 拓扑释放：等待航点空闲
    if (!ctx.release_wait_waypoint.empty()) {
      // dead-end / blocking 场景下，如果阻塞车仍在执行避让链（avoidance/avoidance_return），
      // 不能仅凭拓扑瞬时清空就释放；否则会出现“刚放开又被挡住”的振荡。
      auto is_blocking_wait_reason = [&ctx]() -> bool {
        return ctx.reason.find("blocking") != std::string::npos ||
               ctx.reason.find("dead_end") != std::string::npos ||
               ctx.reason.find("avoidance") != std::string::npos;
      };
      bool peer_avoidance_in_progress = false;
      bool peer_avoidance_stale = false;
      std::string peer_task_id;
      if (is_blocking_wait_reason()) {
        auto peer_nav_it = robot_nav_info_.find(ctx.wait_for_robot);
        if (peer_nav_it != robot_nav_info_.end() && peer_nav_it->second) {
          peer_task_id = peer_nav_it->second->current_task_id;
          if (peer_task_id.rfind("avoidance_", 0) == 0 ||
              peer_task_id.rfind("avoidance_return_", 0) == 0)
          {
            const auto & peer_nav = peer_nav_it->second;
            peer_avoidance_in_progress =
              peer_nav->has_active_goal ||
              peer_nav->pre_rotate_pending ||
              !peer_nav->pending_waypoint_path.empty();
            peer_avoidance_stale = !peer_avoidance_in_progress;
          }
        }
      }
      if (peer_avoidance_stale) {
        auto peer_nav = get_robot_nav_info(ctx.wait_for_robot);
        if (peer_nav) {
          PersistLogger::log_warn(
            "hold.clear_stale_peer_avoidance", robot_id, ctx.resume_task_id,
            "peer " + ctx.wait_for_robot + " stale " + peer_task_id +
              " (goal=0), clear stale internal avoidance state",
            __FILE__, __LINE__, __func__);
          occupancy_manager_->release_reservations(ctx.wait_for_robot);
          reset_through_segment_state(peer_nav);
          peer_nav->pre_rotate_pending = false;
          peer_nav->pending_waypoint_path.clear();
          peer_nav->pending_through_segment_after_rotate.clear();
          peer_nav->current_goal_handle.reset();
          peer_nav->current_pose_goal_handle.reset();
          peer_nav->current_follow_goal_handle.reset();
          peer_nav->has_active_goal = false;
          peer_nav->current_task_id.clear();
          peer_nav->route_waypoints.clear();
          peer_nav->current_waypoint_index = 0;
        }
        invalidate_hold_blocker_avoidance_pair(robot_id, ctx.wait_for_robot);
      }
      if (peer_avoidance_in_progress) {
        const std::string defer_log_key = robot_id + "|" + ctx.wait_for_robot;
        const auto now = this->now();
        auto not_before_it = hold_defer_topology_log_not_before_.find(defer_log_key);
        const bool should_log =
          (not_before_it == hold_defer_topology_log_not_before_.end()) ||
          (now >= not_before_it->second);
        if (should_log) {
          hold_defer_topology_log_not_before_[defer_log_key] =
            now + rclcpp::Duration::from_seconds(hold_defer_topology_log_interval_sec_);
          PersistLogger::log_info(
            "hold.defer_topology_release", robot_id, ctx.resume_task_id,
            "peer " + ctx.wait_for_robot + " still in " +
            peer_task_id +
            ", keep hold until avoidance chain settles",
            __FILE__, __LINE__, __func__);
        }
      } else
      if (occupancy_manager_->is_topology_clear_for(robot_id, ctx.release_wait_waypoint)) {
        to_release.push_back({robot_id, "topology_clear"});
        continue;
      } else {
        // Self-heal: topology not clear but peer is not in avoidance chain.
        // Re-detect the actual blocker of the edge/waypoint we are waiting for, switch wait_for if needed,
        // and try to nudge the blocker away.
        std::string self_wp;
        auto st_it = robot_statuses_.find(robot_id);
        if (st_it != robot_statuses_.end()) {
          self_wp = st_it->second.current_waypoint;
          if (self_wp.empty()) {
            self_wp = traffic_manager_->find_nearest_waypoint(st_it->second.current_pose, 2.5);
          }
        }
        std::string actual_blocker;
        if (!self_wp.empty()) {
          actual_blocker = occupancy_manager_->check_can_enter(robot_id, self_wp, ctx.release_wait_waypoint);
        }
        if (actual_blocker.empty()) {
          actual_blocker = occupancy_manager_->check_waypoint_free(robot_id, ctx.release_wait_waypoint);
        }
        if (!actual_blocker.empty() && actual_blocker != ctx.wait_for_robot) {
          PersistLogger::log_info(
            "hold.wait_for_switch", robot_id, ctx.resume_task_id,
            "switch wait_for " + (ctx.wait_for_robot.empty() ? std::string("-") : ctx.wait_for_robot) +
              " -> " + actual_blocker + " due to topology not clear at " + ctx.release_wait_waypoint,
            __FILE__, __LINE__, __func__);
          ctx.wait_for_robot = actual_blocker;
        }
        if (!actual_blocker.empty() && robot_yieldable_as_idle_blocker(actual_blocker)) {
          std::vector<std::string> exclude = ctx.resume_route;
          std::string avoid_wp = occupancy_manager_->find_avoidance_waypoint(ctx.release_wait_waypoint, exclude);
          if (!avoid_wp.empty()) {
            std::string blocker_wp;
            auto blk_it = robot_statuses_.find(actual_blocker);
            if (blk_it != robot_statuses_.end()) {
              blocker_wp = blk_it->second.current_waypoint;
              if (blocker_wp.empty()) {
                blocker_wp = traffic_manager_->find_nearest_waypoint(blk_it->second.current_pose, 2.5);
              }
            }
            if (!blocker_wp.empty()) {
              auto path = traffic_manager_->find_path(blocker_wp, avoid_wp);
              if (path.size() >= 2) {
                const std::string hop_blocker =
                  occupancy_manager_->check_can_enter(actual_blocker, path[0], path[1]);
                if (!hop_blocker.empty() && hop_blocker == robot_id) {
                  // First-hop is blocked by requester itself.
                  // Try an alternative avoidance waypoint for blocker to avoid
                  // entering a permanent "trigger -> skip -> trigger" loop.
                  std::vector<std::string> alt_exclude = exclude;
                  if (!ctx.release_wait_waypoint.empty()) {
                    alt_exclude.push_back(ctx.release_wait_waypoint);
                  }
                  alt_exclude.push_back(path[1]);
                  std::string alt_wp =
                    occupancy_manager_->find_avoidance_waypoint(blocker_wp, alt_exclude);
                  if (!alt_wp.empty() && alt_wp != avoid_wp) {
                    auto alt_path = traffic_manager_->find_path(blocker_wp, alt_wp);
                    if (alt_path.size() >= 2) {
                      const std::string alt_hop_blocker =
                        occupancy_manager_->check_can_enter(actual_blocker, alt_path[0], alt_path[1]);
                      if (alt_hop_blocker.empty() || alt_hop_blocker != robot_id) {
                        PersistLogger::log_info(
                          "hold.trigger_blocker_avoidance_alt", robot_id, ctx.resume_task_id,
                          "switch blocker " + actual_blocker + " avoidance " + avoid_wp +
                            " -> " + alt_wp + " due to first-hop block by requester " + robot_id,
                          __FILE__, __LINE__, __func__);
                        avoid_wp = alt_wp;
                        path = std::move(alt_path);
                      } else {
                        const auto now = this->now();
                        auto delayed_it = hold_release_delayed_since_.find(robot_id);
                        if (delayed_it == hold_release_delayed_since_.end() ||
                            delayed_it->second.nanoseconds() <= 0)
                        {
                          hold_release_delayed_since_[robot_id] = now;
                          delayed_it = hold_release_delayed_since_.find(robot_id);
                        }
                        const double defer_sec = (now - delayed_it->second).seconds();
                        PersistLogger::log_info(
                          "hold.trigger_blocker_avoidance_skip", robot_id, ctx.resume_task_id,
                          "skip triggering blocker " + actual_blocker + " to " + avoid_wp +
                            ": first hop " + path[0] + "->" + path[1] +
                            " blocked by requester " + robot_id +
                            ", alt=" + alt_wp + " also blocked" +
                            " deferred=" + std::to_string(defer_sec) + "s",
                          __FILE__, __LINE__, __func__);
                        if (defer_sec >= 4.0) {
                          PersistLogger::log_warn(
                            "hold.deadlock_break_fallback", robot_id, ctx.resume_task_id,
                            "skip loop >4s, force deadlock_break release",
                            __FILE__, __LINE__, __func__);
                          to_release.push_back({robot_id, "deadlock_break"});
                        }
                        continue;
                      }
                    }
                  } else {
                    const auto now = this->now();
                    auto delayed_it = hold_release_delayed_since_.find(robot_id);
                    if (delayed_it == hold_release_delayed_since_.end() ||
                        delayed_it->second.nanoseconds() <= 0)
                    {
                      hold_release_delayed_since_[robot_id] = now;
                      delayed_it = hold_release_delayed_since_.find(robot_id);
                    }
                    const double defer_sec = (now - delayed_it->second).seconds();
                    PersistLogger::log_info(
                      "hold.trigger_blocker_avoidance_skip", robot_id, ctx.resume_task_id,
                      "skip triggering blocker " + actual_blocker + " to " + avoid_wp +
                        ": first hop " + path[0] + "->" + path[1] +
                        " immediately blocked by requester " + robot_id +
                        " deferred=" + std::to_string(defer_sec) + "s",
                      __FILE__, __LINE__, __func__);
                    if (defer_sec >= 4.0) {
                      PersistLogger::log_warn(
                        "hold.deadlock_break_fallback", robot_id, ctx.resume_task_id,
                        "skip loop >4s, force deadlock_break release",
                        __FILE__, __LINE__, __func__);
                      to_release.push_back({robot_id, "deadlock_break"});
                    }
                    continue;
                  }
                }
              }
            }
            if (should_trigger_blocker_avoidance(
                  robot_id, actual_blocker, avoid_wp, ctx.resume_task_id, "hold_release_wait"))
            {
              PersistLogger::log_info(
                "hold.trigger_blocker_avoidance", robot_id, ctx.resume_task_id,
                "triggering blocker " + actual_blocker + " to avoidance " + avoid_wp +
                  " (release_wait=" + ctx.release_wait_waypoint + ")",
                __FILE__, __LINE__, __func__);
              const bool force_blocker_move = is_robot_hard_held(actual_blocker);
              request_move_idle_robot_to_avoidance(actual_blocker, avoid_wp, force_blocker_move);
            }
          }
        }
      }
    }

    // Peer 离线释放（peer 不存在或已离线）
    {
      auto peer_it = robot_statuses_.find(ctx.wait_for_robot);
      if (peer_it == robot_statuses_.end() ||
          peer_it->second.connection_status != "online")
      {
        // 如果 hold 原因是等待阻塞者让行（idle_blocking / dead_end），
        // 且 peer 离线了，说明无法通过让行解决，直接标记任务失败
        bool is_blocking_wait = (ctx.reason.find("blocking") != std::string::npos ||
                                  ctx.reason.find("dead_end") != std::string::npos ||
                                  ctx.reason.find("avoidance") != std::string::npos);
        if (is_blocking_wait && !ctx.resume_task_id.empty() &&
            !is_internal_task_id(ctx.resume_task_id))
        {
          // 直接标记任务失败
          RCLCPP_ERROR(
            this->get_logger(),
            "event=hold.fail task=%s robot=%s state_prev=hold state_new=failed reason=PEER_OFFLINE detail=hold_reason:%s peer:%s",
            ctx.resume_task_id.c_str(), robot_id.c_str(), ctx.reason.c_str(),
            ctx.wait_for_robot.c_str());
          task_scheduler_->fail_task(ctx.resume_task_id, "Blocked by offline robot");
          fleet_msgs::msg::TaskInfo ti = task_scheduler_->get_task_info(ctx.resume_task_id);
          task_status_pub_->publish(ti);
          finalize_task_completion(robot_id, ctx.resume_task_id);
          // hold 也需要释放，但 finalize_task_completion 已经清理了
          continue;
        }
        to_release.push_back({robot_id, "peer_offline"});
      }
    }
  }

  for (auto & [robot_id, reason] : to_release) {
    release_hard_hold(robot_id, reason);
  }
}

std::string FleetManagerNode::find_safe_dead_end_retreat_waypoint(
  const std::string & requester_robot_id,
  const std::string & blocker_robot_id,
  const std::string & requester_waypoint,
  const std::string & blocked_waypoint,
  const std::string & requester_destination)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  const auto waypoint_poses = traffic_manager_->get_all_waypoint_poses();
  std::string best_waypoint;
  size_t best_retreat_len = std::numeric_limits<size_t>::max();
  size_t best_resume_len = std::numeric_limits<size_t>::max();
  size_t best_escape_len = std::numeric_limits<size_t>::max();

  for (const auto & [candidate_wp, _pose] : waypoint_poses) {
    if (candidate_wp.empty() || candidate_wp == requester_waypoint || candidate_wp == blocked_waypoint) {
      continue;
    }
    if (!occupancy_manager_->check_waypoint_free(requester_robot_id, candidate_wp).empty()) {
      continue;
    }

    auto retreat_path = traffic_manager_->find_path(requester_waypoint, candidate_wp);
    if (retreat_path.size() < 2) continue;
    if (std::find(retreat_path.begin() + 1, retreat_path.end(), blocked_waypoint) != retreat_path.end()) {
      continue;
    }

    auto resume_route = traffic_manager_->find_path(candidate_wp, requester_destination);
    if (resume_route.size() < 2) continue;

    size_t candidate_escape_len = std::numeric_limits<size_t>::max();
    bool blocker_can_escape = false;
    for (const auto & [escape_wp, _escape_pose] : waypoint_poses) {
      if (escape_wp.empty() || escape_wp == blocked_waypoint || escape_wp == requester_waypoint ||
          escape_wp == candidate_wp)
      {
        continue;
      }
      if (std::find(resume_route.begin(), resume_route.end(), escape_wp) != resume_route.end()) {
        continue;
      }
      if (!occupancy_manager_->check_waypoint_free(blocker_robot_id, escape_wp).empty()) {
        continue;
      }

      auto blocker_escape_path = traffic_manager_->find_path(blocked_waypoint, escape_wp);
      if (blocker_escape_path.size() < 2) continue;
      if (std::find(blocker_escape_path.begin() + 1, blocker_escape_path.end(), candidate_wp) !=
          blocker_escape_path.end())
      {
        continue;
      }

      blocker_can_escape = true;
      candidate_escape_len = std::min(candidate_escape_len, blocker_escape_path.size());
    }

    if (!blocker_can_escape) {
      PersistLogger::log_info(
        "sched.dead_end_retreat_reject", requester_robot_id, "",
        "reject retreat_wp=" + candidate_wp +
        " requester=" + requester_waypoint +
        " blocked=" + blocked_waypoint +
        " because blocker has no escape path",
        __FILE__, __LINE__, __func__);
      continue;
    }

    if (retreat_path.size() < best_retreat_len ||
        (retreat_path.size() == best_retreat_len && resume_route.size() < best_resume_len) ||
        (retreat_path.size() == best_retreat_len && resume_route.size() == best_resume_len &&
         candidate_escape_len < best_escape_len))
    {
      best_waypoint = candidate_wp;
      best_retreat_len = retreat_path.size();
      best_resume_len = resume_route.size();
      best_escape_len = candidate_escape_len;
    }
  }

  if (!best_waypoint.empty()) {
    PersistLogger::log_info(
      "sched.dead_end_retreat_select", requester_robot_id, "",
      "requester=" + requester_waypoint +
      " blocked=" + blocked_waypoint +
      " choose retreat_wp=" + best_waypoint +
      " retreat_len=" + std::to_string(best_retreat_len) +
      " resume_len=" + std::to_string(best_resume_len) +
      " escape_len=" + std::to_string(best_escape_len),
      __FILE__, __LINE__, __func__);
  }

  return best_waypoint;
}

}  // namespace fleet_manager
