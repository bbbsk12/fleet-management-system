#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#include <tf2/LinearMath/Quaternion.hpp>

namespace fleet_manager
{
namespace
{
constexpr double kNavControlCommitSec = 0.9;
}

FleetManagerNode::BlockingDecision FleetManagerNode::decide_blocking_resolution(
  const std::string & requester_robot_id,
  const std::string & task_id,
  const std::string & from_wp,
  const std::string & blocked_wp,
  const std::string & destination_wp,
  const std::vector<std::string> & requester_route,
  const std::string & blocker_robot_id,
  const std::string & log_tag_prefix)
{
  BlockingDecision decision;
  std::string avoidance_wp = occupancy_manager_->find_avoidance_waypoint(blocked_wp, requester_route);
  // Avoidance path must not pass through requester position, otherwise this is effectively dead-end.
  if (!avoidance_wp.empty()) {
    auto blocker_avoidance_path = traffic_manager_->find_path(blocked_wp, avoidance_wp);
    for (size_t pi = 1; pi < blocker_avoidance_path.size(); ++pi) {
      if (blocker_avoidance_path[pi] == from_wp) {
        PersistLogger::log_info(
          log_tag_prefix + ".avoidance_path_blocked", requester_robot_id, task_id,
          "avoid_wp=" + avoidance_wp + " path goes through requester at " + from_wp +
            ", treating as dead-end",
          __FILE__, __LINE__, __func__);
        avoidance_wp.clear();
        break;
      }
    }
  }

  if (!avoidance_wp.empty()) {
    decision.type = BlockingDecisionType::HOLD_AND_AVOID;
    decision.avoidance_waypoint = avoidance_wp;
    return decision;
  }

  if (is_robot_hard_held(requester_robot_id)) {
    return decision;
  }

  const std::string retreat_wp = find_safe_dead_end_retreat_waypoint(
    requester_robot_id, blocker_robot_id, from_wp, blocked_wp, destination_wp);
  if (retreat_wp.empty()) {
    return decision;
  }
  decision.type = BlockingDecisionType::RETREAT_AND_HOLD;
  decision.retreat_waypoint = retreat_wp;
  return decision;
}

bool FleetManagerNode::apply_blocking_resolution(
  const BlockingDecision & decision,
  const std::string & requester_robot_id,
  const std::string & task_id,
  const std::string & from_wp,
  const std::string & blocked_wp,
  const std::string & destination_wp,
  const std::string & blocker_robot_id,
  bool use_dispatch_requests,
  const std::string & log_tag_prefix)
{
  if (decision.type == BlockingDecisionType::NONE) return false;
  if (decision.type == BlockingDecisionType::FAIL_TASK) {
    if (task_id.empty() || is_internal_task_id(task_id)) return false;
    RCLCPP_ERROR(
      this->get_logger(),
      "event=conflict.fail task=%s robot=%s state_prev=in_progress "
      "state_new=failed reason=%s detail=blocker:%s",
      task_id.c_str(),
      requester_robot_id.c_str(),
      decision.fail_reason.empty() ? "FAILED" : decision.fail_reason.c_str(),
      blocker_robot_id.empty() ? "-" : blocker_robot_id.c_str());
    PersistLogger::log_error(
      log_tag_prefix + ".fail", requester_robot_id, task_id,
      "task failed reason=" + (decision.fail_reason.empty() ? std::string("FAILED") : decision.fail_reason) +
        " blocker=" + (blocker_robot_id.empty() ? std::string("-") : blocker_robot_id),
      __FILE__, __LINE__, __func__);
    task_scheduler_->fail_task(task_id, decision.fail_reason.empty() ? "Failed" : decision.fail_reason);
    fleet_msgs::msg::TaskInfo ti = task_scheduler_->get_task_info(task_id);
    task_status_pub_->publish(ti);
    finalize_task_completion(requester_robot_id, task_id);
    return true;
  }
  if (decision.type == BlockingDecisionType::HOLD_AND_AVOID) {
    if (!is_robot_hard_held(requester_robot_id)) {
      if (use_dispatch_requests) {
        request_apply_hard_hold(
          requester_robot_id, "idle_blocking_wait_clear", blocker_robot_id, false, true);
      } else {
        apply_hard_hold(
          requester_robot_id, "idle_blocking_wait_clear", blocker_robot_id, false, true);
      }
      hold_contexts_[requester_robot_id].release_wait_waypoint = blocked_wp;
    }
    request_move_idle_robot_to_avoidance(
      blocker_robot_id, decision.avoidance_waypoint, /*force=*/false);
    return true;
  }
  if (decision.type == BlockingDecisionType::FORCE_YIELD_BLOCKER) {
    if (!is_robot_hard_held(requester_robot_id)) {
      request_apply_hard_hold(requester_robot_id, "dead_end_force_wait", blocker_robot_id, false, true);
      hold_contexts_[requester_robot_id].release_wait_waypoint = blocked_wp;
    }
    request_move_idle_robot_to_avoidance(
      blocker_robot_id, decision.avoidance_waypoint, /*force=*/true);
    return true;
  }
  if (decision.type == BlockingDecisionType::DEADLOCK_YIELD_SELF) {
    const std::string actor_id =
      decision.actor_robot_id.empty() ? requester_robot_id : decision.actor_robot_id;
    if (decision.avoidance_waypoint.empty()) return false;
    if (decision.release_hold_first && is_robot_hard_held(actor_id)) {
      request_release_hard_hold(actor_id, "deadlock_break");
    }
    request_move_idle_robot_to_avoidance(
      actor_id, decision.avoidance_waypoint, /*force=*/false);
    robot_avoidance_cooldown_until_[actor_id] =
      this->now() + rclcpp::Duration::from_seconds(avoidance_cooldown_sec_);
    return true;
  }
  if (decision.type != BlockingDecisionType::RETREAT_AND_HOLD) return false;

  PersistLogger::log_info(
    log_tag_prefix + ".dead_end_retreat", requester_robot_id, task_id,
    "blocker=" + blocker_robot_id + " at " + blocked_wp +
      " no avoidance, requester retreats to " + decision.retreat_waypoint,
    __FILE__, __LINE__, __func__);

  if (use_dispatch_requests) {
    request_stage_safe_hold(
      requester_robot_id, decision.retreat_waypoint, "dead_end_requester_retreat", blocker_robot_id, false,
      /*pending_trigger_avoidance_robot=*/blocker_robot_id,
      /*pending_trigger_avoidance_from=*/from_wp, true);
  } else {
    stage_safe_hold(
      requester_robot_id, decision.retreat_waypoint, "dead_end_requester_retreat", blocker_robot_id, false,
      /*pending_trigger_avoidance_robot=*/blocker_robot_id,
      /*pending_trigger_avoidance_from=*/from_wp, true);
  }

  auto retreat_to_dest = traffic_manager_->find_path(decision.retreat_waypoint, destination_wp);
  auto & hctx = hold_contexts_[requester_robot_id];
  if (!retreat_to_dest.empty()) {
    hctx.resume_route = retreat_to_dest;
  }
  if (hctx.resume_task_id.empty()) {
    hctx.resume_task_id = task_id;
  }
  hctx.release_wait_waypoint = blocked_wp;
  return true;
}

FleetManagerNode::BlockingDecision FleetManagerNode::decide_offline_blocker_failure(
  const std::string & /*requester_robot_id*/,
  const std::string & task_id,
  const std::string & blocker_robot_id,
  double blocked_sec,
  double timeout_sec,
  const std::string & log_tag_prefix)
{
  BlockingDecision d;
  if (task_id.empty() || is_internal_task_id(task_id)) return d;
  if (blocked_sec < timeout_sec) return d;
  d.type = BlockingDecisionType::FAIL_TASK;
  d.fail_reason = "Blocked by offline robot";
  PersistLogger::log_error(
    log_tag_prefix + ".offline_block_timeout", "", task_id,
    "blocker=" + blocker_robot_id + " blocked_sec=" + std::to_string(blocked_sec),
    __FILE__, __LINE__, __func__);
  return d;
}

FleetManagerNode::BlockingDecision FleetManagerNode::decide_deadlock_break_resolution(
  const std::vector<std::string> & cycle_robot_ids,
  const rclcpp::Time & now,
  const std::string & log_tag_prefix)
{
  BlockingDecision d;
  if (cycle_robot_ids.size() < 2) return d;

  std::string victim;
  size_t shortest_route = std::numeric_limits<size_t>::max();
  for (const auto & rid : cycle_robot_ids) {
    auto ni = get_robot_nav_info(rid);
    if (!ni) continue;
    if (ni->route_waypoints.size() < shortest_route) {
      shortest_route = ni->route_waypoints.size();
      victim = rid;
    }
  }
  if (victim.empty()) return d;

  auto cooldown_it = robot_avoidance_cooldown_until_.find(victim);
  if (cooldown_it != robot_avoidance_cooldown_until_.end() && now < cooldown_it->second) {
    return d;
  }

  auto v_st = robot_statuses_.find(victim);
  std::string v_wp = (v_st != robot_statuses_.end()) ? v_st->second.current_waypoint : "";
  if (v_wp.empty()) {
    auto v_nav = get_robot_nav_info(victim);
    if (v_nav && !v_nav->route_waypoints.empty()) v_wp = v_nav->route_waypoints[0];
  }
  if (v_wp.empty()) return d;

  std::vector<std::string> exclude;
  for (const auto & rid : cycle_robot_ids) {
    auto ni = get_robot_nav_info(rid);
    if (ni) exclude.insert(exclude.end(), ni->route_waypoints.begin(), ni->route_waypoints.end());
  }
  const std::string avoid = occupancy_manager_->find_avoidance_waypoint(v_wp, exclude);
  if (avoid.empty()) return d;

  RCLCPP_WARN(
    this->get_logger(),
    "event=deadlock.resolve task=- robot=%s state_prev=blocked state_new=yielding "
    "reason=DEADLOCK_CYCLE detail=cycle_size:%zu avoidance_wp:%s",
    victim.c_str(), cycle_robot_ids.size(), avoid.c_str());
  PersistLogger::log_warn(
    log_tag_prefix + ".deadlock.resolve", victim, "",
    "cycle_size=" + std::to_string(cycle_robot_ids.size()) +
      " avoidance_wp=" + avoid,
    __FILE__, __LINE__, __func__);

  d.type = BlockingDecisionType::DEADLOCK_YIELD_SELF;
  d.actor_robot_id = victim;
  d.avoidance_waypoint = avoid;
  d.release_hold_first = is_robot_hard_held(victim);
  return d;
}

FleetManagerNode::BlockingDecision FleetManagerNode::decide_force_yield_dead_end_blocker(
  const std::string & requester_robot_id,
  const std::string & task_id,
  const std::string & from_wp,
  const std::string & blocked_wp,
  const std::string & blocker_robot_id,
  const std::vector<std::string> & requester_route,
  const rclcpp::Time & now,
  const std::string & log_tag_prefix)
{
  BlockingDecision d;
  auto cooldown_it = robot_avoidance_cooldown_until_.find(blocker_robot_id);
  if (cooldown_it != robot_avoidance_cooldown_until_.end() && now < cooldown_it->second) {
    return d;
  }

  std::string avoid_wp = occupancy_manager_->find_avoidance_waypoint(blocked_wp, requester_route);
  if (avoid_wp.empty()) return d;

  auto blocker_avoid_path = traffic_manager_->find_path(blocked_wp, avoid_wp);
  for (size_t pi = 1; pi < blocker_avoid_path.size(); ++pi) {
    if (blocker_avoid_path[pi] == from_wp) {
      return d;
    }
  }

  bool blocker_busy_or_settling = false;
  auto blocker_nav = get_robot_nav_info(blocker_robot_id);
  if (blocker_nav) {
    const rclcpp::Time activity_ref =
      (blocker_nav->nav_last_activity.nanoseconds() > 0) ? blocker_nav->nav_last_activity :
      (blocker_nav->last_goal_issue_time.nanoseconds() > 0) ? blocker_nav->last_goal_issue_time :
      blocker_nav->nav_active_since;
    const double activity_idle_sec =
      (activity_ref.nanoseconds() > 0) ? (now - activity_ref).seconds() : 0.0;
    constexpr double kBusyActivityStaleSec = 2.5;
    const bool stale_busy_flag =
      blocker_nav->has_active_goal &&
      !blocker_nav->pre_rotate_pending &&
      blocker_nav->pending_waypoint_path.empty() &&
      blocker_nav->pending_through_segment_after_rotate.empty() &&
      activity_ref.nanoseconds() > 0 &&
      activity_idle_sec >= kBusyActivityStaleSec;
    const bool soon_active =
      blocker_nav->has_active_goal ||
      blocker_nav->pre_rotate_pending ||
      !blocker_nav->pending_waypoint_path.empty() ||
      !blocker_nav->pending_through_segment_after_rotate.empty();
    const bool in_settling =
      blocker_nav->recent_cancel_until.nanoseconds() > 0 &&
      now < blocker_nav->recent_cancel_until;
    const bool in_commit =
      blocker_nav->nav_control_commit_until.nanoseconds() > 0 &&
      now < blocker_nav->nav_control_commit_until;
    blocker_busy_or_settling = (soon_active && !stale_busy_flag) || in_settling || in_commit;
    if (stale_busy_flag) {
      PersistLogger::log_warn(
        log_tag_prefix + ".dead_end_force_yield_stale_busy", requester_robot_id, task_id,
        "blocker " + blocker_robot_id + " has stale active-goal flag (" +
          std::to_string(activity_idle_sec) + "s idle), allowing force_yield",
        __FILE__, __LINE__, __func__);
    }
  }
  if (blocker_busy_or_settling) {
    PersistLogger::log_info(
      log_tag_prefix + ".dead_end_force_yield_defer", requester_robot_id, task_id,
      "defer force_yield: blocker " + blocker_robot_id + " is busy/settling",
      __FILE__, __LINE__, __func__);
    return d;
  }

  auto rst_it = robot_statuses_.find(requester_robot_id);
  if (rst_it != robot_statuses_.end() && rst_it->second.location_type == "segment") {
    PersistLogger::log_info(
      log_tag_prefix + ".dead_end_force_yield_defer", requester_robot_id, task_id,
      "defer force_yield: requester is on segment",
      __FILE__, __LINE__, __func__);
    return d;
  }

  d.type = BlockingDecisionType::FORCE_YIELD_BLOCKER;
  d.avoidance_waypoint = avoid_wp;
  d.force = true;
  PersistLogger::log_warn(
    log_tag_prefix + ".dead_end_force_yield", requester_robot_id, task_id,
    "blocker=" + blocker_robot_id + " at " + blocked_wp +
      " is on mission but blocking dead-end, force yielding to " + avoid_wp,
    __FILE__, __LINE__, __func__);
  return d;
}

bool FleetManagerNode::handle_blocking_with_unified_policy(
  const std::string & requester_robot_id,
  const std::string & task_id,
  const std::string & from_wp,
  const std::string & blocked_wp,
  const std::string & destination_wp,
  const std::vector<std::string> & requester_route,
  const std::string & blocker_robot_id,
  bool use_dispatch_requests,
  const std::string & log_tag_prefix)
{
  const auto decision = decide_blocking_resolution(
    requester_robot_id, task_id, from_wp, blocked_wp, destination_wp, requester_route,
    blocker_robot_id, log_tag_prefix);
  return apply_blocking_resolution(
    decision, requester_robot_id, task_id, from_wp, blocked_wp, destination_wp, blocker_robot_id,
    use_dispatch_requests, log_tag_prefix);
}

bool FleetManagerNode::fail_task_if_offline_blocker_timeout(
  const std::string & requester_robot_id,
  const std::string & task_id,
  const std::string & blocker_robot_id,
  double blocked_sec,
  double timeout_sec)
{
  const auto d = decide_offline_blocker_failure(
    requester_robot_id, task_id, blocker_robot_id, blocked_sec, timeout_sec, "conflict");
  return apply_blocking_resolution(
    d, requester_robot_id, task_id, /*from_wp=*/"", /*blocked_wp=*/"", /*destination_wp=*/"",
    blocker_robot_id, /*use_dispatch_requests=*/true, "conflict");
}

bool FleetManagerNode::try_force_yield_dead_end_blocker(
  const std::string & requester_robot_id,
  const std::string & task_id,
  const std::string & from_wp,
  const std::string & blocked_wp,
  const std::string & blocker_robot_id,
  const std::vector<std::string> & requester_route,
  const rclcpp::Time & now)
{
  const auto d = decide_force_yield_dead_end_blocker(
    requester_robot_id, task_id, from_wp, blocked_wp, blocker_robot_id, requester_route, now, "conflict");
  return apply_blocking_resolution(
    d, requester_robot_id, task_id, from_wp, blocked_wp, blocked_wp, blocker_robot_id,
    /*use_dispatch_requests=*/true, "conflict");
}

bool FleetManagerNode::move_idle_robot_to_avoidance(
  const std::string & idle_robot_id,
  const std::string & avoidance_waypoint,
  bool force)
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  auto status_it = robot_statuses_.find(idle_robot_id);
  if (status_it == robot_statuses_.end()) return false;

  auto nav_info = get_robot_nav_info(idle_robot_id);
  if (!nav_info) return false;

  // 避让深度限制：如果机器人已经在执行避让任务，不再叠加新的避让
  // 但在 force 模式（死胡同强制让行 / 死锁打破）下允许打断，否则会出现“卡在航线中”无法自救。
  if (!nav_info->current_task_id.empty() && is_internal_task_id(nav_info->current_task_id) && !force) {
    PersistLogger::log_info(
      "avoidance.depth_limit", idle_robot_id, nav_info->current_task_id,
      "skip avoidance to " + avoidance_waypoint + ": already in avoidance task",
      __FILE__, __LINE__, __func__);
    return false;
  }

  // 避让级联防护：检查避让代数，超过上限则拒绝
  {
    auto gen_it = robot_avoidance_generation_.find(idle_robot_id);
    int current_gen = (gen_it != robot_avoidance_generation_.end()) ? gen_it->second : 0;
    if (current_gen >= max_avoidance_generation_) {
      PersistLogger::log_warn(
        "avoidance.cascade_limit", idle_robot_id, "",
        "skip avoidance: generation " + std::to_string(current_gen) +
        " >= max " + std::to_string(max_avoidance_generation_),
        __FILE__, __LINE__, __func__);
      return false;
    }
  }

  // Bug-fix: hard-held 的底盘不应被派去避让
  // 但在 force 模式下允许打断 hard-hold，否则链式调度可能永久停滞在航线中。
  if (is_robot_hard_held(idle_robot_id) && !force) {
    PersistLogger::log_warn(
      "avoidance", idle_robot_id, nav_info->current_task_id,
      "skip avoidance: robot is hard-held",
      __FILE__, __LINE__, __func__);
    return false;
  }
  if (is_robot_hard_held(idle_robot_id) && force) {
    auto h_it = hold_contexts_.find(idle_robot_id);
    if (h_it != hold_contexts_.end()) {
      const auto & hctx = h_it->second;
      // 保护用户任务：内部避让不应强行打断“用户任务 hold”。
      // 否则会出现你日志里的链路：A 等待 B 清路 -> B 避让失败 -> 反向强拆 A 的 hold。
      if (!hctx.resume_task_id.empty() && !is_internal_task_id(hctx.resume_task_id)) {
        // 例外：交通清障等待型的 hold 本质上就是为了让行/解阻塞。
        // 若 force 仍被 veto，会出现双方互锁（日志里的 “first_hop_blocked + hold_release_wait” 闭环）。
        const bool traffic_clear_hold =
          (hctx.reason == "idle_blocking_wait_clear") ||
          (hctx.reason.rfind("idle_blocking", 0) == 0);
        if (!traffic_clear_hold) {
        PersistLogger::log_warn(
          "avoidance.force_break_hold_veto", idle_robot_id, hctx.resume_task_id,
          "veto force break hard-hold: robot is holding for user task, target=" + avoidance_waypoint,
          __FILE__, __LINE__, __func__);
        return false;
        }
      }
    }
    PersistLogger::log_warn(
      "avoidance.force_break_hold", idle_robot_id, nav_info->current_task_id,
      "force avoidance breaks hard-hold to " + avoidance_waypoint,
      __FILE__, __LINE__, __func__);
    // 释放预约避免幽灵锁；并移除 hold 上下文，允许本次避让接管
    occupancy_manager_->release_reservations(idle_robot_id);
    hold_release_delayed_since_.erase(idle_robot_id);
    hold_contexts_.erase(idle_robot_id);
  }

  const bool user_mission_nav =
    !nav_info->current_task_id.empty() && !is_internal_task_id(nav_info->current_task_id) &&
    (nav_info->has_active_goal || nav_info->pre_rotate_pending || !nav_info->pending_waypoint_path.empty());
  // force 模式：死胡同强制避让时，允许中断用户任务（会保存恢复状态）
  if (user_mission_nav && !force) return false;

  // force 模式下保存被中断的用户任务，以便避让完成后恢复
  if (force && user_mission_nav) {
    auto & hctx = hold_contexts_[idle_robot_id];
    hctx.resume_task_id = nav_info->current_task_id;
    hctx.resume_route = nav_info->route_waypoints;
    // 取消当前导航目标
    cancel_active_robot_goals(nav_info);
    publish_zero_cmd_vel(idle_robot_id, 10);
    PersistLogger::log_info(
      "avoidance.force_save_task", idle_robot_id, nav_info->current_task_id,
      "saving user task for resumption after forced avoidance",
      __FILE__, __LINE__, __func__);
  }

  // Round 12e: 不要打断正在执行的内部导航（避让/回归）
  if (!nav_info->current_task_id.empty() && is_internal_task_id(nav_info->current_task_id) &&
      (nav_info->has_active_goal || nav_info->pre_rotate_pending) && !force) {
    PersistLogger::log_info(
      "avoidance", idle_robot_id, nav_info->current_task_id,
      "skip avoidance: already executing internal nav task",
      __FILE__, __LINE__, __func__);
    return false;
  }

  // 冷却
  auto cd_it = robot_avoidance_cooldown_until_.find(idle_robot_id);
  if (cd_it != robot_avoidance_cooldown_until_.end() && this->now() < cd_it->second) return false;

  std::string nearest_wp = traffic_manager_->find_nearest_waypoint(status_it->second.current_pose, 2.5);
  if (nearest_wp.empty()) return false;
  if (nearest_wp == avoidance_waypoint) {
    PersistLogger::log_info(
      "avoidance.noop", idle_robot_id, nav_info->current_task_id,
      "skip avoidance: already at waypoint " + nearest_wp,
      __FILE__, __LINE__, __func__);
    // 若这是一次 staged hold（如 dead_end_requester_retreat）引发的“退让到某航点”，
    // 且我们已经判定机器人就在该航点附近，则无需等待更严格的 acceptance radius
    // 触发 promote（可能因定位噪声/半径过小而永远不触发）。直接就地激活 hold，
    // 并触发等待链（触发阻塞者避让），避免任务停滞。
    {
      auto h_it = hold_contexts_.find(idle_robot_id);
      if (h_it != hold_contexts_.end() && !h_it->second.active &&
          h_it->second.pending_hold_waypoint == avoidance_waypoint)
      {
        auto ctx = h_it->second;  // copy; apply_hard_hold will mutate map entry
        PersistLogger::log_info(
          "hold.promote_at_waypoint", idle_robot_id, ctx.resume_task_id,
          "promote staged hold at " + avoidance_waypoint +
            " (reason=" + ctx.reason + ", wait_for=" + ctx.wait_for_robot + ")",
          __FILE__, __LINE__, __func__);

        apply_hard_hold(
          idle_robot_id,
          ctx.reason.empty() ? "arrived_avoidance" : ctx.reason,
          ctx.wait_for_robot,
          ctx.release_when_peer_far,
          ctx.snapshot_resume);

        // 死角退避：申请车已到达退让点，立刻触发阻塞者避让
        if (!ctx.pending_trigger_avoidance_robot.empty() &&
            !ctx.pending_trigger_avoidance_from.empty())
        {
          std::string trigger_target;
          if (!ctx.resume_route.empty()) {
            trigger_target = occupancy_manager_->find_avoidance_waypoint(
              ctx.pending_trigger_avoidance_from, ctx.resume_route);
          }
          if (trigger_target.empty()) {
            trigger_target = ctx.pending_trigger_avoidance_from;
          }
          PersistLogger::log_info(
            "hold.trigger_blocker_avoidance", idle_robot_id, ctx.resume_task_id,
            "triggering " + ctx.pending_trigger_avoidance_robot +
              " to avoidance " + trigger_target +
              " (from=" + ctx.pending_trigger_avoidance_from + ")",
            __FILE__, __LINE__, __func__);
          const bool force_trigger = is_robot_hard_held(ctx.pending_trigger_avoidance_robot);
          request_move_idle_robot_to_avoidance(
            ctx.pending_trigger_avoidance_robot, trigger_target, force_trigger);
        }
      }
    }
    // force 模式下如果“已在避让点”，也需要主动唤醒等待本车的对端，
    // 否则会出现双方都不动（对端保持 hold，本车早退）。
    if (force) {
      std::vector<std::string> waiting_peers;
      waiting_peers.reserve(hold_contexts_.size());
      for (const auto & [rid, hctx] : hold_contexts_) {
        if (hctx.active && hctx.wait_for_robot == idle_robot_id) {
          waiting_peers.push_back(rid);
        }
      }
      for (const auto & rid : waiting_peers) {
        request_release_hard_hold(rid, "force_avoidance_noop_retry");
      }
    }
    return true;
  }

  auto build_extended_exclude = [&](const std::vector<std::string> & base) {
    std::vector<std::string> ex = base;
    // 额外排除：其它机器人正在执行的活跃路线航点（避免把避让车派进“车流中间”）
    for (const auto & [oid, onav] : robot_nav_info_) {
      if (!onav || oid == idle_robot_id) continue;
      if (!onav->has_active_goal) continue;
      for (const auto & wp : onav->route_waypoints) ex.push_back(wp);
    }
    // 去重
    std::sort(ex.begin(), ex.end());
    ex.erase(std::unique(ex.begin(), ex.end()), ex.end());
    return ex;
  };

  auto try_candidate = [&](const std::string & candidate_wp) -> bool {
    auto avoidance_path = traffic_manager_->find_path(nearest_wp, candidate_wp);
    if (avoidance_path.empty()) return false;
    if (avoidance_path.size() <= 1) {
      PersistLogger::log_info(
        "avoidance.noop", idle_robot_id, nav_info->current_task_id,
        "skip avoidance candidate " + candidate_wp + ": path_len=" +
        std::to_string(avoidance_path.size()),
        __FILE__, __LINE__, __func__);
      return false;
    }

    // Round 14c: 检查避让路径是否与其它正在导航的机器人的路线航段对冲
    // force 模式用于打破僵局，预检查只作参考；真正安全由 reserve_next_hop 兜底。
    if (!force) {
      std::set<std::string> avoid_edges;
      for (size_t i = 0; i + 1 < avoidance_path.size(); ++i) {
        const auto & a = avoidance_path[i];
        const auto & b = avoidance_path[i + 1];
        avoid_edges.insert(a < b ? (a + "|" + b) : (b + "|" + a));
      }
      for (const auto & [oid, onav] : robot_nav_info_) {
        if (!onav || oid == idle_robot_id) continue;
        if (!onav->has_active_goal && onav->current_task_id.empty()) continue;
        if (onav->route_waypoints.size() < 2) continue;
        // held-idle 的旧路线不参与冲突判定
        {
          auto hold_it = hold_contexts_.find(oid);
          if (hold_it != hold_contexts_.end() && hold_it->second.active &&
              !onav->has_active_goal) {
            continue;
          }
        }
        // held waiting for this robot: skip
        {
          auto hold_it = hold_contexts_.find(oid);
          if (hold_it != hold_contexts_.end() && hold_it->second.active &&
              hold_it->second.wait_for_robot == idle_robot_id) {
            continue;
          }
        }
        for (size_t j = 0; j + 1 < onav->route_waypoints.size(); ++j) {
          const auto & oa = onav->route_waypoints[j];
          const auto & ob = onav->route_waypoints[j + 1];
          const std::string oek = oa < ob ? (oa + "|" + ob) : (ob + "|" + oa);
          if (avoid_edges.count(oek)) {
            PersistLogger::log_warn(
              "avoidance.path_conflict", idle_robot_id, "",
              "avoidance to " + candidate_wp + " conflicts with " + oid +
              " route edge " + oa + "->" + ob,
              __FILE__, __LINE__, __func__);
            return false;
          }
        }
      }
    }

    // Round 19: 目的航点不能落在其它活跃路线航点上
    if (!force) {
      for (const auto & [oid, onav] : robot_nav_info_) {
        if (!onav || oid == idle_robot_id) continue;
        if (onav->route_waypoints.size() < 2) continue;
        if (!onav->has_active_goal && onav->current_task_id.empty()) continue;
        {
          auto hold_it = hold_contexts_.find(oid);
          if (hold_it != hold_contexts_.end() && hold_it->second.active &&
              !onav->has_active_goal) {
            continue;
          }
        }
        {
          auto hold_it = hold_contexts_.find(oid);
          if (hold_it != hold_contexts_.end() && hold_it->second.active &&
              hold_it->second.wait_for_robot == idle_robot_id) {
            continue;
          }
        }
        for (size_t wi = 1; wi < onav->route_waypoints.size(); ++wi) {
          if (onav->route_waypoints[wi] == candidate_wp) {
            PersistLogger::log_warn(
              "avoidance.dest_on_route", idle_robot_id, "",
              "avoidance dest " + candidate_wp + " is on " + oid +
              " route wp[" + std::to_string(wi) + "]",
              __FILE__, __LINE__, __func__);
            return false;
          }
        }
      }
    }

    // 使用通过检查的候选
    avoidance_return_waypoint_[idle_robot_id] = nearest_wp;
    nav_info->route_waypoints = avoidance_path;
    nav_info->current_waypoint_index = 0;
    nav_info->current_task_id = "avoidance_" + idle_robot_id;
    robot_avoidance_cooldown_until_[idle_robot_id] =
      this->now() + rclcpp::Duration::from_seconds(avoidance_cooldown_sec_);
    return true;
  };

  // 优先尝试调用者给定的 avoidance waypoint；失败则尝试一个“更保守”的替代点（排除其它活跃路线）
  std::string chosen_wp = avoidance_waypoint;
  if (!try_candidate(chosen_wp)) {
    std::vector<std::string> base_exclude = {nearest_wp};
    auto ex = build_extended_exclude(base_exclude);
    std::string fallback = occupancy_manager_->find_avoidance_waypoint(nearest_wp, ex);
    if (!fallback.empty() && fallback != chosen_wp) {
      PersistLogger::log_info(
        "avoidance.fallback", idle_robot_id, "",
        "fallback avoidance from=" + nearest_wp + " requested=" + chosen_wp + " -> " + fallback,
        __FILE__, __LINE__, __func__);
      chosen_wp = fallback;
    }
    if (!try_candidate(chosen_wp)) {
      PersistLogger::log_warn(
        "avoidance.force_failed", idle_robot_id, nav_info->current_task_id,
        "force=" + std::string(force ? "1" : "0") +
        " failed to find executable avoidance path from " + nearest_wp +
        " requested=" + avoidance_waypoint + " fallback=" + chosen_wp,
        __FILE__, __LINE__, __func__);
      if (force) {
        // force 也失败时，主动释放“等待本车让路”的对端 hold，避免双方永久静止。
        std::vector<std::string> waiting_peers;
        waiting_peers.reserve(hold_contexts_.size());
        for (const auto & [rid, hctx] : hold_contexts_) {
          if (hctx.active && hctx.wait_for_robot == idle_robot_id) {
            waiting_peers.push_back(rid);
          }
        }
        for (const auto & rid : waiting_peers) {
          request_release_hard_hold(rid, "force_avoidance_failed_retry");
        }
      }
      return false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Moving idle robot %s to avoidance %s",
              idle_robot_id.c_str(), chosen_wp.c_str());
  PersistLogger::log_info(
    "avoidance", idle_robot_id, "",
    "moving to avoidance_wp=" + chosen_wp + " from=" + nearest_wp +
    " path_len=" + std::to_string(nav_info->route_waypoints.size()),
    __FILE__, __LINE__, __func__);

  // 避让也属于活跃导航：刷新活动时间戳，避免被 nav.stuck_recover 误判为“旧目标卡死”
  nav_info->nav_active_since = this->now();
  nav_info->nav_last_activity = nav_info->nav_active_since;

  // 递增避让代数
  {
    auto gen_it = robot_avoidance_generation_.find(idle_robot_id);
    if (gen_it != robot_avoidance_generation_.end()) {
      gen_it->second++;
    } else {
      robot_avoidance_generation_[idle_robot_id] = 1;
    }
    PersistLogger::log_info(
      "avoidance.generation", idle_robot_id, "",
      "avoidance generation now " + std::to_string(robot_avoidance_generation_[idle_robot_id]),
      __FILE__, __LINE__, __func__);
  }

  // 避让导航也需要段前对正：计算首跳转角，过大时先 pre-rotate 再走
  const auto & active_avoid_path = nav_info->route_waypoints;
  if (active_avoid_path.size() >= 2 && status_it != robot_statuses_.end()) {
    const auto & cur_pose = status_it->second.current_pose;
    // 检查位置数据有效性
    const bool pose_valid = !(std::abs(cur_pose.position.x) < 1e-6 && std::abs(cur_pose.position.y) < 1e-6);
    if (pose_valid) {
      const double cur_yaw = get_yaw_from_quat(cur_pose.orientation);
      const double desired_yaw = bearing_pose_to_waypoint(cur_pose, active_avoid_path[1]);
      const double turn_err = std::abs(normalize_angle_rad(desired_yaw - cur_yaw));
      const bool need_pre_rotate =
        pre_rotate_every_segment_ || (turn_err > pre_rotate_turn_threshold_rad_);
      if (need_pre_rotate) {
        if (nav_info->recent_cancel_until.nanoseconds() > 0 &&
            this->now() < nav_info->recent_cancel_until) {
          // 取消沉降期内不要再下发原地旋转，避免 Nav2 在 goal cancel/replace 竞争时崩溃。
          navigate_to_next_waypoint(idle_robot_id);
          return true;
        }
        // 用 NavigateToPose 原地转向
        cancel_active_robot_goals(nav_info);
        const uint64_t cmd_seq = ++nav_info->nav_command_seq;

        nav_info->pre_rotate_pending = true;
        nav_info->pending_through_segment_after_rotate.clear();
        nav_info->pending_waypoint_path = active_avoid_path;
        nav_info->nav_active_since = this->now();
        nav_info->nav_last_activity = nav_info->nav_active_since;

        geometry_msgs::msg::PoseStamped rotate_goal;
        rotate_goal.header.frame_id = "map";
        rotate_goal.header.stamp = this->now();
        rotate_goal.pose = cur_pose;  // 位置保持不变
        tf2::Quaternion q;
        q.setRPY(0, 0, desired_yaw);
        rotate_goal.pose.orientation.x = q.x();
        rotate_goal.pose.orientation.y = q.y();
        rotate_goal.pose.orientation.z = q.z();
        rotate_goal.pose.orientation.w = q.w();

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = rotate_goal;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.goal_response_callback =
          [this, idle_robot_id, cmd_seq](GoalHandleNavigateToPose::SharedPtr goal_handle) {
            std::lock_guard<std::recursive_mutex> lock(state_mutex_);
            auto ni = get_robot_nav_info(idle_robot_id);
            if (!ni || ni->nav_command_seq != cmd_seq) return;
            if (!goal_handle) {
              RCLCPP_WARN(this->get_logger(),
                "Avoidance pre-rotate NavigateToPose rejected for %s, falling back", idle_robot_id.c_str());
              // Root-cause fix:
              // Goal rejected but has_active_goal was set true before async_send_goal.
              // If we don't clear it here, robot can remain in fake active-goal forever.
              ni->has_active_goal = false;
              ni->current_pose_goal_handle.reset();
              ni->nav_active_since = rclcpp::Time{};
              ni->nav_last_activity = rclcpp::Time{};
              ni->pre_rotate_pending = false;
              auto path = ni->pending_waypoint_path;
              ni->pending_waypoint_path.clear();
              if (!path.empty()) {
                ni->route_waypoints = path;
                ni->current_waypoint_index = 0;
                navigate_to_next_waypoint(idle_robot_id);
              }
            } else {
              ni->current_pose_goal_handle = goal_handle;
              ni->nav_last_activity = this->now();
            }
          };

        send_goal_options.feedback_callback =
          [this, idle_robot_id, cmd_seq](GoalHandleNavigateToPose::SharedPtr,
                           const std::shared_ptr<const NavigateToPose::Feedback>) {
            std::lock_guard<std::recursive_mutex> lock(state_mutex_);
            auto ni = get_robot_nav_info(idle_robot_id);
            if (!ni || ni->nav_command_seq != cmd_seq) return;
            ni->nav_last_activity = this->now();
          };

        send_goal_options.result_callback =
          [this, idle_robot_id, cmd_seq](const GoalHandleNavigateToPose::WrappedResult & result) {
            std::lock_guard<std::recursive_mutex> lock(state_mutex_);
            auto ni = get_robot_nav_info(idle_robot_id);
            if (!ni || ni->nav_command_seq != cmd_seq) return;
            ni->has_active_goal = false;
            ni->current_pose_goal_handle.reset();
            ni->pre_rotate_pending = false;
            ni->nav_active_since = rclcpp::Time{};
            ni->nav_last_activity = rclcpp::Time{};

            auto path = ni->pending_waypoint_path;
            ni->pending_waypoint_path.clear();

            PersistLogger::log_info(
              "avoidance.pre_rotate", idle_robot_id, "",
              "Pre-rotate result=" + std::to_string(static_cast<int>(result.code)),
              __FILE__, __LINE__, __func__);

            if (!path.empty()) {
              ni->route_waypoints = path;
              ni->current_waypoint_index = 0;
              navigate_to_next_waypoint(idle_robot_id);
            }
          };

        const bool pose_ready = action_client_ready<NavigateToPose>(nav_info->nav_client);
        if (!nav_info->nav_client || !pose_ready) {
          RCLCPP_WARN(this->get_logger(),
            "NavigateToPose not ready for avoidance pre-rotate %s, skipping", idle_robot_id.c_str());
          nav_info->pre_rotate_pending = false;
          navigate_to_next_waypoint(idle_robot_id);
          return true;
        }

        nav_info->has_active_goal = true;
        nav_info->last_goal_issue_time = this->now();
        nav_info->nav_control_commit_until =
          nav_info->last_goal_issue_time + rclcpp::Duration::from_seconds(kNavControlCommitSec);
        nav_info->nav_client->async_send_goal(goal_msg, send_goal_options);

        PersistLogger::log_info(
          "avoidance.pre_rotate", idle_robot_id, "",
          std::string("Pre-rotate requested: mode=") +
          (pre_rotate_every_segment_ ? "every_segment" : "threshold") +
          " turn=" + std::to_string(turn_err * 180.0 / M_PI) + "deg",
          __FILE__, __LINE__, __func__);
        return true;
      }
    }
  }

  navigate_to_next_waypoint(idle_robot_id);
  return true;
}

}  // namespace fleet_manager
