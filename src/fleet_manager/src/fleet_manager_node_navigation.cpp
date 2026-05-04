#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>

namespace fleet_manager
{

// ==================== Chain Retreat Constants ====================

namespace
{
constexpr int    kMaxChainDepth = 5;
constexpr double kChainTotalTimeout = 180.0;  // more headroom for deep chains
constexpr double kChainStepTimeout = 30.0;
constexpr int    kMaxChainStepRetries = 2;
constexpr int    kMinDepthForParallelRetreat = 3;  // deep chains get parallel leaf steps
constexpr const char * kChainTaskPrefix = "chain_retreat_";
}  // namespace

// ==================== Navigation Entry ====================

bool FleetManagerNode::start_navigation(
  const std::string & robot_id,
  const std::string & target_wp,
  const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);
  if (!ni) { scheduler_->fail_task(task_id, "no nav client"); return false; }

  const auto now = this->now();

  if (ni->has_active_goal || !ni->route.empty() || !ni->current_task_id.empty()) {
    scheduler_->mark_task_waiting(task_id);
    return false;
  }

  if (ni->retry_after.nanoseconds() > 0 && now < ni->retry_after) {
    scheduler_->mark_task_waiting(task_id);
    return false;
  }

  auto st = robots_.find(robot_id);
  if (st == robots_.end()) { scheduler_->fail_task(task_id, "robot not found"); return false; }

  // already at target?
  auto target_pose = traffic_->get_waypoint_pose(target_wp);
  double dx = st->second.current_pose.position.x - target_pose.position.x;
  double dy = st->second.current_pose.position.y - target_pose.position.y;
  if (std::hypot(dx, dy) <= waypoint_radius_) {
    on_nav_succeeded(robot_id, task_id);
    return true;
  }

  // find nearest waypoint as start
  std::string start_wp;
  double min_d = std::numeric_limits<double>::max();
  for (const auto & [id, pose] : traffic_->get_all_waypoint_poses()) {
    double d = std::hypot(
      st->second.current_pose.position.x - pose.position.x,
      st->second.current_pose.position.y - pose.position.y);
    if (d < min_d) { min_d = d; start_wp = id; }
  }

  // verify the robot owns the start waypoint zone
  if (!start_wp.empty()) {
    std::string holder = occupancy_->get_zone_holder(start_wp);
    if (!holder.empty() && holder != robot_id) {
      PersistLogger::log_warn("nav.start_zone_missing", robot_id, task_id,
        "nearest wp " + start_wp + " held by " + holder + ", retrying",
        __FILE__, __LINE__, __func__);
      ni->retry_after = now + rclcpp::Duration::from_seconds(0.5);
      scheduler_->mark_task_waiting(task_id);
      return false;
    }
    if (holder.empty()) {
      // no one holds this zone — verify robot is physically near it
      auto start_pose = traffic_->get_waypoint_pose(start_wp);
      double d2 = std::hypot(
        st->second.current_pose.position.x - start_pose.position.x,
        st->second.current_pose.position.y - start_pose.position.y);
      if (d2 > waypoint_radius_ * 2.5) {
        PersistLogger::log_warn("nav.start_too_far", robot_id, task_id,
          "nearest wp " + start_wp + " is " + std::to_string(d2) + "m away, defer",
          __FILE__, __LINE__, __func__);
        ni->retry_after = now + rclcpp::Duration::from_seconds(0.5);
        scheduler_->mark_task_waiting(task_id);
        return false;
      }
    }
  }

  // weighted path finding
  std::vector<std::string> path;
  if (!start_wp.empty() && start_wp != target_wp) {
    auto occ_zones = occupancy_->get_occupied_zones();
    std::set<std::string> occ_edges;
    // convert zone occupancy to edge occupancy for routing
    for (const auto & [wp, conns] : traffic_->get_adjacency_map()) {
      if (occ_zones.count(wp)) {
        for (const auto & nb : conns) {
          occ_edges.insert(wp < nb ? (wp + "|" + nb) : (nb + "|" + wp));
        }
      }
    }
    path = traffic_->find_path_weighted(start_wp, target_wp, occ_edges, occ_zones);
    if (path.empty())
      path = traffic_->find_path(start_wp, target_wp);
  }
  if (path.empty()) path.push_back(target_wp);

  // dedup consecutive
  std::vector<std::string> clean;
  for (const auto & w : path) {
    if (clean.empty() || clean.back() != w) clean.push_back(w);
  }
  path = clean;
  if (path.empty()) path.push_back(target_wp);

  // check first hop is clear
  if (path.size() >= 2) {
    std::string blocker = occupancy_->can_enter(robot_id, path[0], path[1]);
    if (!blocker.empty()) {
      PersistLogger::log_info("nav.first_hop_blocked", robot_id, task_id,
        "from=" + path[0] + " to=" + path[1] + " blocker=" + blocker,
        __FILE__, __LINE__, __func__);

      // blocked by offline robot → fail task
      auto bs = robots_.find(blocker);
      if (bs == robots_.end() || bs->second.connection_status != "online") {
        scheduler_->fail_task(task_id, "blocked by offline robot " + blocker);
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
        task_pub_->publish(ti);
        return false;
      }

      // retry with backoff
      ni->retry_count++;
      // Per-robot jitter: prevents simultaneous retry of multiple blocked robots
double jitter = 0.7 + 0.6 * (static_cast<double>(std::hash<std::string>{}(robot_id) % 1000) / 1000.0);
double backoff = jitter * retry_base_ * std::pow(1.5, std::min(ni->retry_count, retry_max_));
      ni->retry_after = now + rclcpp::Duration::from_seconds(backoff);
      scheduler_->mark_task_waiting(task_id);
      return false;
    }
  }

  // reserve first hop (if multi-waypoint) or verify single-waypoint zone
  if (path.size() >= 2) {
    if (!occupancy_->reserve_next(robot_id, path[0], path[1])) {
      ni->retry_count++;
      // Per-robot jitter: prevents simultaneous retry of multiple blocked robots
double jitter = 0.7 + 0.6 * (static_cast<double>(std::hash<std::string>{}(robot_id) % 1000) / 1000.0);
double backoff = jitter * retry_base_ * std::pow(1.5, std::min(ni->retry_count, retry_max_));
      ni->retry_after = now + rclcpp::Duration::from_seconds(backoff);
      scheduler_->mark_task_waiting(task_id);
      return false;
    }
  } else {
    // single-waypoint path: verify zone is free before moving
    std::string wp = path[0];
    std::string blocker = occupancy_->waypoint_blocker(robot_id, wp);
    if (!blocker.empty()) {
      PersistLogger::log_info("nav.single_wp_blocked", robot_id, task_id,
        "wp=" + wp + " blocker=" + blocker, __FILE__, __LINE__, __func__);
      ni->retry_count++;
      // Per-robot jitter: prevents simultaneous retry of multiple blocked robots
double jitter = 0.7 + 0.6 * (static_cast<double>(std::hash<std::string>{}(robot_id) % 1000) / 1000.0);
double backoff = jitter * retry_base_ * std::pow(1.5, std::min(ni->retry_count, retry_max_));
      ni->retry_after = now + rclcpp::Duration::from_seconds(backoff);
      scheduler_->mark_task_waiting(task_id);
      return false;
    }
  }

  ni->route       = path;
  ni->route_index = 0;
  ni->current_task_id = task_id;
  ni->retry_count = 0;
  ni->retry_after = rclcpp::Time{};

  RCLCPP_INFO(this->get_logger(),
    "nav.start task=%s robot=%s path=%zu waypoints: %s",
    task_id.c_str(), robot_id.c_str(), path.size(), join_waypoints(path).c_str());
  PersistLogger::log_info("nav.start", robot_id, task_id,
    "path=" + join_waypoints(path), __FILE__, __LINE__, __func__);

  scheduler_->mark_task_navigating(task_id);
  navigate_to_next_waypoint(robot_id);
  return true;
}

// ==================== Waypoint-by-waypoint ====================

void FleetManagerNode::navigate_to_next_waypoint(const std::string & robot_id)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);
  if (!ni || ni->route.empty()) return;

  if (ni->has_active_goal) return;  // already moving
  if (ni->current_task_id.empty()) return;

  size_t n = ni->route.size();
  if (ni->route_index >= n) return;

  auto st = robots_.find(robot_id);

  // advance past any waypoint the robot has already physically reached
  // (use raw pose, not stale discretized location_type, to handle the gap
  //  between NavigateToPose result callback and the next update_location)
  size_t target = ni->route_index;
  if (st != robots_.end()) {
    while (target < n) {
      auto wp_pose = traffic_->get_waypoint_pose(ni->route[target]);
      double d = std::hypot(
        st->second.current_pose.position.x - wp_pose.position.x,
        st->second.current_pose.position.y - wp_pose.position.y);
      if (d <= waypoint_radius_) {
        target++;
      } else {
        break;
      }
    }
  }
  ni->route_index = target;
  if (ni->route_index >= n) return;

  std::string wp     = ni->route[target];
  bool is_final      = (target == n - 1);

  // reserve this hop if not already at the waypoint
  if (target > 0 && target < n) {
    std::string from = ni->route[target - 1];
    std::string blocker = occupancy_->can_enter(robot_id, from, wp);
    if (!blocker.empty()) {
      PersistLogger::log_info("nav.hop_blocked", robot_id, ni->current_task_id,
        "from=" + from + " to=" + wp + " blocker=" + blocker,
        __FILE__, __LINE__, __func__);

      // wait with backoff
      ni->retry_count++;
      // Per-robot jitter: prevents simultaneous retry of multiple blocked robots
double jitter = 0.7 + 0.6 * (static_cast<double>(std::hash<std::string>{}(robot_id) % 1000) / 1000.0);
double backoff = jitter * retry_base_ * std::pow(1.5, std::min(ni->retry_count, retry_max_));
      ni->retry_after = this->now() + rclcpp::Duration::from_seconds(backoff);

      // Blocker coordination: handle both idle AND stationary-but-blocked robots
      if (ni->retry_count >= 2 && !is_internal_task_id(ni->current_task_id)) {
        auto blocker_ni = get_or_create_nav(blocker);
        auto blocker_st = robots_.find(blocker);
        bool blocker_online = blocker_st != robots_.end() &&
          blocker_st->second.connection_status == "online";
        bool blocker_stationary = blocker_ni && is_robot_stationary(blocker);

        if (blocker_stationary && blocker_online) {
          bool mutual = is_mutual_block(blocker, wp, robot_id, from);
          if (mutual) {
            // Guard: only one chain at a time
            if (chain_plan_.active) {
              PersistLogger::log_info("nav.chain_busy", robot_id, ni->current_task_id,
                "another chain already active, deferring",
                __FILE__, __LINE__, __func__);
              // Extend backoff to wait for the active chain to resolve this blocker
              ni->retry_after = this->now() + rclcpp::Duration::from_seconds(5.0);
              return;
            }

            // Save blocker's task info before chain overwrites nav state
            chain_plan_.saved_task_ids.clear();
            chain_plan_.saved_targets.clear();
            if (!blocker_ni->current_task_id.empty()) {
              chain_plan_.saved_task_ids[blocker] = blocker_ni->current_task_id;
              if (!blocker_ni->route.empty())
                chain_plan_.saved_targets[blocker] = blocker_ni->route.back();
            }
            if (!ni->current_task_id.empty()) {
              chain_plan_.saved_task_ids[robot_id] = ni->current_task_id;
              if (!ni->route.empty())
                chain_plan_.saved_targets[robot_id] = ni->route.back();
            }

            std::set<std::string> blocked_set(ni->route.begin(), ni->route.end());
            if (try_build_retreat_chain(robot_id, from, wp, blocker, blocked_set, 0)) {
              // Pre-validation: verify all chain targets are valid
              bool chain_valid = true;
              for (const auto & s : chain_plan_.steps) {
                auto test = traffic_->get_waypoint_pose(s.target_wp);
                if (test.position.x == 0.0 && test.position.y == 0.0 && test.position.z == 0.0) {
                  auto all = traffic_->get_all_waypoint_poses();
                  if (all.find(s.target_wp) == all.end()) {
                    PersistLogger::log_warn("nav.chain_invalid_wp", s.robot_id, "",
                      "waypoint " + s.target_wp + " not in map",
                      __FILE__, __LINE__, __func__);
                    chain_valid = false;
                    break;
                  }
                }
              }
              if (!chain_valid) {
                chain_plan_.steps.clear();
                chain_plan_.saved_task_ids.clear();
                chain_plan_.saved_targets.clear();
              } else {
                chain_plan_.original_requester = robot_id;
                chain_plan_.original_target = ni->route.back();
                chain_plan_.original_task_id = ni->current_task_id;
                chain_plan_.active = true;
                chain_plan_.started_at = this->now();
                chain_plan_.current_step = 0;
                chain_plan_.step_retry_count = 0;

                // Pause requester's nav while chain executes
                ni->has_active_goal = false;
                ni->route.clear();
                ni->route_index = 0;
                ni->retry_count = 0;

                // Also pause blocker's nav
                blocker_ni->has_active_goal = false;
                blocker_ni->route.clear();
                blocker_ni->route_index = 0;
                blocker_ni->retry_count = 0;

                PersistLogger::log_info("nav.chain_started", robot_id, ni->current_task_id,
                  "chain retreat initiated, " + std::to_string(chain_plan_.steps.size()) + " steps",
                  __FILE__, __LINE__, __func__);

                execute_chain_step();
                return;
              }
            }
            chain_plan_.saved_task_ids.clear();
            chain_plan_.saved_targets.clear();
          }

          // Simple avoidance: only for truly idle blockers (no task)
          bool blocker_truly_idle = blocker_ni &&
            blocker_ni->current_task_id.empty() &&
            blocker_ni->route.empty();
          if (!mutual && blocker_truly_idle) {
            std::string avoid_wp = occupancy_->find_nearest_free_waypoint(wp);
            if (!avoid_wp.empty() && avoid_wp != wp) {
              PersistLogger::log_info("nav.avoid_idle_blocker", robot_id, ni->current_task_id,
                "asking idle blocker " + blocker + " to move from " + wp + " to " + avoid_wp,
                __FILE__, __LINE__, __func__);
              blocker_ni->route = {avoid_wp};
              blocker_ni->route_index = 0;
              blocker_ni->current_task_id = "avoidance_" + blocker;
              blocker_ni->retry_count = 0;
              blocker_ni->retry_after = rclcpp::Time{};
              navigate_to_waypoint(blocker, avoid_wp, blocker_ni->current_task_id, true);
            }
          }
        }
      }

      if (ni->retry_count > retry_max_ * 2) {
        // give up and requeue
        PersistLogger::log_warn("nav.give_up", robot_id, ni->current_task_id,
          "max retries exceeded, requeueing task",
          __FILE__, __LINE__, __func__);
        std::string tid = ni->current_task_id;
        occupancy_->release_reservations(robot_id);
        ni->current_task_id.clear();
        ni->route.clear();
        ni->route_index = 0;
        ni->retry_count = 0;
        if (scheduler_->would_exceed_retry_cycles(tid, max_task_retry_cycles_)) {
          scheduler_->fail_task(tid, "hop blocked, max retry cycles exceeded");
          fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
          task_pub_->publish(ti);
          finalize_task_completion(robot_id, tid);
        } else {
          scheduler_->mark_task_pending_preserve(tid);
          fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
          task_pub_->publish(ti);
        }
      }
      return;
    }

    if (!occupancy_->reserve_next(robot_id, from, wp)) {
      ni->retry_count++;
      ni->retry_after = this->now() + rclcpp::Duration::from_seconds(retry_base_);
      return;
    }
    ni->retry_count = 0;  // successful reservation resets retries
  }

  ni->route_index = target;
  navigate_to_waypoint(robot_id, wp, ni->current_task_id, is_final);
}

void FleetManagerNode::navigate_to_waypoint(
  const std::string & robot_id,
  const std::string & wp_id,
  const std::string & task_id,
  bool is_final)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);
  if (!ni) return;

  const auto now = this->now();
  if (ni->recent_cancel_until.nanoseconds() > 0 && now < ni->recent_cancel_until) return;

  // cancel old goal
  if (ni->goal_handle && ni->nav_client) {
    ni->nav_client->async_cancel_goal(ni->goal_handle);
    ni->goal_handle.reset();
    ni->has_active_goal = false;
    ni->nav_seq++;
  }

  auto wp_pose = traffic_->get_waypoint_pose(wp_id);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = now;
  goal.pose.position = wp_pose.position;

  // orientation: face toward next waypoint if not final
  if (!is_final && ni->route.size() > ni->route_index + 1) {
    auto next_pose = traffic_->get_waypoint_pose(ni->route[ni->route_index + 1]);
    double yaw = std::atan2(
      next_pose.position.y - wp_pose.position.y,
      next_pose.position.x - wp_pose.position.x);
    tf2::Quaternion q; q.setRPY(0, 0, yaw);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();
  } else {
    goal.pose.orientation = wp_pose.orientation;
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = goal;

  auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  uint64_t seq = ni->nav_seq;
  opts.goal_response_callback =
    [this, robot_id, seq](GoalHandleNavigate::SharedPtr gh) {
      std::lock_guard<std::recursive_mutex> l(mtx_);
      auto n = get_or_create_nav(robot_id);
      if (!n || n->nav_seq != seq) return;
      if (!gh) {
        n->has_active_goal = false;
        PersistLogger::log_warn("nav.rejected", robot_id, n->current_task_id,
          "NavigateToPose rejected", __FILE__, __LINE__, __func__);
      } else {
        n->goal_handle = gh;
        n->nav_last_activity = this->now();
      }
    };

  opts.result_callback =
    [this, robot_id, task_id, is_final, seq](const GoalHandleNavigate::WrappedResult & r) {
      std::lock_guard<std::recursive_mutex> l(mtx_);
      auto n = get_or_create_nav(robot_id);
      if (!n || n->nav_seq != seq) return;

      n->has_active_goal = false;
      n->goal_handle.reset();

      if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
        n->retry_count = 0;
        // Chain step completion
        if (chain_plan_.active && task_id.rfind(kChainTaskPrefix, 0) == 0) {
          on_chain_step_complete(robot_id, true);
          return;
        }
        if (is_final) {
          on_nav_succeeded(robot_id, task_id);
        } else {
          navigate_to_next_waypoint(robot_id);
        }
      } else if (r.code == rclcpp_action::ResultCode::CANCELED) {
        // cancelled intentionally, do nothing
      } else {
        // aborted or failed: retry
        PersistLogger::log_warn("nav.failed", robot_id, task_id,
          "nav result code=" + std::to_string(static_cast<int>(r.code)),
          __FILE__, __LINE__, __func__);
        n->retry_count++;
        if (n->retry_count <= 3) {
          navigate_to_next_waypoint(robot_id);
        } else {
          std::string tid = task_id;
          occupancy_->release_reservations(robot_id);
          n->current_task_id.clear();
          n->route.clear();
          n->route_index = 0;
          n->retry_count = 0;
          if (scheduler_->would_exceed_retry_cycles(tid, max_task_retry_cycles_)) {
            scheduler_->fail_task(tid, "nav failed repeatedly, max retry cycles exceeded");
            fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
            if (!ti.task_id.empty()) task_pub_->publish(ti);
            finalize_task_completion(robot_id, tid);
          } else {
            scheduler_->mark_task_pending_preserve(tid);
            fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
            if (!ti.task_id.empty()) task_pub_->publish(ti);
          }
        }
      }
    };

  if (!ni->nav_client || !ni->nav_client->action_server_is_ready()) {
    PersistLogger::log_warn("nav.not_ready", robot_id, task_id,
      "NavigateToPose server not ready", __FILE__, __LINE__, __func__);
    ni->retry_after = now + rclcpp::Duration::from_seconds(0.5);
    return;
  }

  ni->has_active_goal = true;
  ni->nav_since = now;
  ni->nav_last_activity = now;
  ni->recent_cancel_until = now + rclcpp::Duration::from_seconds(kNavCancelSettlingSec);

  ni->nav_client->async_send_goal(goal_msg, opts);
}

// ==================== Arrival ====================

void FleetManagerNode::on_nav_succeeded(
  const std::string & robot_id, const std::string & task_id)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);

  // Internal tasks (avoidance, chain retreat, auto-relocate): just clean up nav
  if (task_id.rfind("avoidance_", 0) == 0 ||
      task_id.rfind(kChainTaskPrefix, 0) == 0 ||
      task_id.rfind("relocate_", 0) == 0) {
    if (ni) {
      occupancy_->release_reservations(robot_id);
      ni->current_task_id.clear();
      ni->has_active_goal = false;
      ni->route.clear();
      ni->route_index = 0;
    }
    return;
  }

  uint8_t type = scheduler_->get_task_type(task_id);

  if (type == 1 || type == 0) {
    // CRUISE: done
    scheduler_->complete_task(task_id);
    fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
    task_pub_->publish(ti);
    finalize_task_completion(robot_id, task_id);
  } else {
    // LOAD/UNLOAD/SITE_SPECIFIC: send to chassis
    scheduler_->mark_task_executing(task_id);
    fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
    task_pub_->publish(ti);

    auto info = scheduler_->get_task_info(task_id);
    uint16_t wp_num = wp_to_u16(info.waypoint_id);
    send_chassis_cmd(robot_id, task_id, wp_num, type, info.site_code);
  }
}

void FleetManagerNode::check_arrivals()
{
  // Chain timeout check
  if (chain_plan_.active) {
    const auto now = this->now();
    if ((now - chain_plan_.started_at).seconds() >= kChainTotalTimeout) {
      abort_chain("chain total timeout (" + std::to_string(kChainTotalTimeout) + "s)");
      return;
    }
  }

  // waypoint arrival is handled by NavigateToPose result callbacks
  // this method provides a backup: if robot is close to target but callback
  // hasn't fired, force arrival
  const auto now = this->now();
  for (auto & [rid, ni] : navs_) {
    if (!ni || !ni->has_active_goal) continue;
    if (ni->route.empty()) continue;

    // Skip internal tasks (chain, avoidance, relocate) — they have their own lifecycle
    if (!ni->current_task_id.empty() && is_internal_task_id(ni->current_task_id)) continue;
    if (ni->current_task_id.rfind(kChainTaskPrefix, 0) == 0) continue;
    if (ni->current_task_id.rfind("relocate_", 0) == 0) continue;

    // check stuck navigation
    if (ni->nav_since.nanoseconds() > 0 && nav_stuck_timeout_ > 0 &&
        (now - ni->nav_since).seconds() > nav_stuck_timeout_) {
      PersistLogger::log_warn("nav.stuck", rid, ni->current_task_id,
        "stuck for " + std::to_string((now - ni->nav_since).seconds()) + "s",
        __FILE__, __LINE__, __func__);
      cancel_goals(ni);
      ni->has_active_goal = false;
      ni->retry_count++;
      if (ni->retry_count <= 3) {
        navigate_to_next_waypoint(rid);
      } else {
        std::string tid = ni->current_task_id;
        occupancy_->release_reservations(rid);
        ni->current_task_id.clear();
        ni->route.clear();
        ni->route_index = 0;
        ni->retry_count = 0;
        if (scheduler_->would_exceed_retry_cycles(tid, max_task_retry_cycles_)) {
          scheduler_->fail_task(tid, "stuck navigation, max retry cycles exceeded");
          fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
          task_pub_->publish(ti);
          finalize_task_completion(rid, tid);
        } else {
          scheduler_->mark_task_pending_preserve(tid);
          fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
          task_pub_->publish(ti);
        }
      }
    }

    // absolute timeout
    if (ni->nav_since.nanoseconds() > 0 && nav_absolute_timeout_ > 0 &&
        (now - ni->nav_since).seconds() > nav_absolute_timeout_) {
      PersistLogger::log_warn("nav.absolute_timeout", rid, ni->current_task_id,
        "absolute timeout", __FILE__, __LINE__, __func__);
      cancel_goals(ni);
      ni->has_active_goal = false;
      std::string tid = ni->current_task_id;
      occupancy_->release_reservations(rid);
      ni->current_task_id.clear();
      ni->route.clear();
      ni->route_index = 0;
      if (scheduler_->would_exceed_retry_cycles(tid, max_task_retry_cycles_)) {
        scheduler_->fail_task(tid, "nav absolute timeout, max retry cycles exceeded");
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
        task_pub_->publish(ti);
        finalize_task_completion(rid, tid);
      } else {
        scheduler_->mark_task_pending_preserve(tid);
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
        task_pub_->publish(ti);
      }
    }
  }
}

// ==================== Chassis ====================

void FleetManagerNode::send_chassis_cmd(
  const std::string & robot_id, const std::string & task_id,
  uint16_t wp_num, uint8_t type, uint32_t site)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);
  if (!ni || !ni->task_cmd_pub) return;

  uint64_t task_num = std::hash<std::string>{}(task_id);
  fleet_msgs::msg::TaskCmd cmd;
  cmd.task_id     = task_num;
  cmd.waypoint_id = wp_num;
  cmd.task_type   = type;
  cmd.site_code   = site;
  cmd.ack         = false;

  ni->task_cmd_pub->publish(cmd);
  ni->chassis_task_sent  = true;
  ni->chassis_handshake_ok = false;
  ni->chassis_acked      = false;
  ni->chassis_hs_deadline = this->now() + rclcpp::Duration::from_seconds(chassis_hs_timeout_);
  ni->chassis_exec_deadline = this->now() + rclcpp::Duration::from_seconds(chassis_hs_timeout_ + chassis_exec_timeout_);
  ni->pending_task_type = type;
  ni->pending_site_code = site;
  ni->pending_wp_num    = wp_num;
  ni->pending_task_num  = task_num;

  PersistLogger::log_info("chassis.send", robot_id, task_id,
    "cmd sent wp=" + std::to_string(wp_num), __FILE__, __LINE__, __func__);
}

void FleetManagerNode::send_chassis_ack(const std::string & robot_id, uint64_t task_num)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);
  if (!ni || !ni->task_cmd_pub) return;

  fleet_msgs::msg::TaskCmd ack;
  ack.task_id = task_num;
  ack.ack = true;
  ni->task_cmd_pub->publish(ack);
}

void FleetManagerNode::chassis_fb_callback(
  const std::string & robot_id, const fleet_msgs::msg::TaskFb::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  auto ni = get_or_create_nav(robot_id);
  if (!ni) return;

  const uint64_t fb_num = msg->task_id;
  const uint8_t status  = msg->status;

  // handle resent feedback for completed tasks
  if (!ni->chassis_task_sent) {
    if (ni->chassis_acked && (status == 1 || status == 2)) {
      send_chassis_ack(robot_id, fb_num);
    }
    return;
  }

  // resent handshake
  if (status == 0 && ni->chassis_handshake_ok) {
    send_chassis_ack(robot_id, fb_num);
    return;
  }

  if (fb_num != ni->pending_task_num) {
    send_chassis_ack(robot_id, fb_num);
    return;
  }

  std::string task_id = ni->current_task_id;

  switch (status) {
    case 0: {  // HANDSHAKE_OK
      ni->chassis_handshake_ok = true;
      ni->chassis_retries = 0;
      send_chassis_ack(robot_id, fb_num);
      PersistLogger::log_info("chassis.handshake", robot_id, task_id,
        "ok", __FILE__, __LINE__, __func__);
      break;
    }
    case 1: {  // COMPLETED
      send_chassis_ack(robot_id, fb_num);
      ni->chassis_task_sent = false;
      ni->chassis_handshake_ok = false;
      ni->chassis_retries = 0;
      ni->chassis_acked = true;
      scheduler_->complete_task(task_id);
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
      task_pub_->publish(ti);
      finalize_task_completion(robot_id, task_id);
      PersistLogger::log_info("chassis.complete", robot_id, task_id,
        "done", __FILE__, __LINE__, __func__);
      break;
    }
    case 2: {  // ERROR
      send_chassis_ack(robot_id, fb_num);
      ni->chassis_task_sent = false;
      ni->chassis_handshake_ok = false;
      ni->chassis_acked = true;
      std::string reason = "chassis error 0x" + std::to_string(msg->error_code);
      scheduler_->fail_task(task_id, reason);
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
      task_pub_->publish(ti);
      finalize_task_completion(robot_id, task_id);
      PersistLogger::log_error("chassis.error", robot_id, task_id,
        reason, __FILE__, __LINE__, __func__);
      break;
    }
    default: break;
  }
}

void FleetManagerNode::chassis_timeout_check()
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  const auto now = this->now();

  for (auto & [rid, ni] : navs_) {
    if (!ni || !ni->chassis_task_sent) continue;

    std::string task_id = ni->current_task_id;

    if (!ni->chassis_handshake_ok) {
      if (now > ni->chassis_hs_deadline) {
        ni->chassis_retries++;
        if (ni->chassis_retries <= chassis_max_retries_) {
          RCLCPP_WARN(this->get_logger(), "chassis hs timeout retry %d/%d for %s",
            ni->chassis_retries, chassis_max_retries_, rid.c_str());
          send_chassis_cmd(rid, task_id, ni->pending_wp_num, ni->pending_task_type, ni->pending_site_code);
        } else {
          RCLCPP_ERROR(this->get_logger(), "chassis hs timeout exhausted for %s", rid.c_str());
          ni->chassis_task_sent = false;
          ni->chassis_handshake_ok = false;
          ni->chassis_retries = 0;
          ni->chassis_acked = true;
          scheduler_->fail_task(task_id, "chassis handshake timeout");
          fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
          task_pub_->publish(ti);
          finalize_task_completion(rid, task_id);
        }
      }
      continue;
    }

    if (now > ni->chassis_exec_deadline) {
      RCLCPP_ERROR(this->get_logger(), "chassis exec timeout for %s", rid.c_str());
      ni->chassis_task_sent = false;
      ni->chassis_handshake_ok = false;
      ni->chassis_acked = true;
      scheduler_->fail_task(task_id, "chassis execution timeout");
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
      task_pub_->publish(ti);
      finalize_task_completion(rid, task_id);
    }
  }
}

bool FleetManagerNode::is_robot_executing(const std::string & robot_id) const
{
  auto it = navs_.find(robot_id);
  return (it != navs_.end() && it->second && it->second->chassis_task_sent);
}

void FleetManagerNode::finalize_task_completion(
  const std::string & robot_id, const std::string & /*task_id*/)
{
  auto ni = get_or_create_nav(robot_id);
  if (!ni) return;
  occupancy_->release_reservations(robot_id);
  ni->current_task_id.clear();
  ni->has_active_goal = false;
  ni->route.clear();
  ni->route_index = 0;
  ni->retry_count = 0;
  ni->chassis_task_sent = false;
  ni->chassis_handshake_ok = false;
  ni->chassis_retries = 0;

  // Auto-relocate from dead-end waypoints to prevent idle blocking
  auto st = robots_.find(robot_id);
  if (st == robots_.end() || st->second.connection_status != "online") return;

  std::string current_wp = st->second.current_waypoint;
  if (current_wp.empty())
    current_wp = traffic_->find_nearest_waypoint(st->second.current_pose);

  auto conns = traffic_->get_waypoint_connections(current_wp);
  if (conns.size() <= 1 && !conns.empty()) {
    // Dead-end waypoint: auto-relocate to the connected waypoint if free
    std::string exit_wp = conns[0];
    if (occupancy_->waypoint_blocker(robot_id, exit_wp).empty()) {
      PersistLogger::log_info("nav.auto_relocate", robot_id, "",
        "relocating from dead-end " + current_wp + " to " + exit_wp,
        __FILE__, __LINE__, __func__);
      ni->route = {exit_wp};
      ni->route_index = 0;
      ni->current_task_id = "relocate_" + robot_id;
      ni->retry_count = 0;
      ni->retry_after = rclcpp::Time{};
      navigate_to_waypoint(robot_id, exit_wp, ni->current_task_id, true);
    }
  }
}

uint16_t FleetManagerNode::wp_to_u16(const std::string & wp_id) const
{
  std::string nums;
  for (auto it = wp_id.rbegin(); it != wp_id.rend(); ++it) {
    if (std::isdigit(*it)) nums.insert(nums.begin(), *it);
    else if (!nums.empty()) break;
  }
  if (nums.empty()) return static_cast<uint16_t>(std::hash<std::string>{}(wp_id) & 0xFFFF);
  return static_cast<uint16_t>(std::stoul(nums) & 0xFFFF);
}

// ==================== LED ====================

uint8_t FleetManagerNode::determine_led_state(const std::string & robot_id) const
{
  auto ni = navs_.find(robot_id);
  if (ni == navs_.end() || !ni->second) return 3;  // IDLE

  if (ni->second->chassis_task_sent) return 2;  // TASK_EXECUTING
  if (ni->second->has_active_goal)  return 0;  // WALKING
  if (!ni->second->current_task_id.empty()) return 1;  // TRAFFIC_WAIT
  return 3;  // IDLE
}

void FleetManagerNode::led_timer_callback()
{
  for (auto & [rid, ni] : navs_) {
    if (!ni || !ni->led_pub) continue;
    auto st = robots_.find(rid);
    if (st == robots_.end() || st->second.connection_status != "online") continue;

    uint8_t s = determine_led_state(rid);
    if (s != ni->last_led_state) {
      fleet_msgs::msg::LEDTask msg;
      msg.state = s;
      ni->led_pub->publish(msg);
      ni->last_led_state = s;
    }
  }
}

void FleetManagerNode::led_status_callback(
  const std::string & robot_id, const fleet_msgs::msg::LEDStatus::SharedPtr msg)
{
  auto ni = get_or_create_nav(robot_id);
  if (!ni) return;
  ni->chassis_led = msg->state;
  ni->led_received = msg->received;
}

// ==================== Helpers ====================

void FleetManagerNode::cancel_goals(const std::shared_ptr<RobotNavInfo> & ni)
{
  if (!ni) return;
  ni->recent_cancel_until = this->now() + rclcpp::Duration::from_seconds(kNavCancelSettlingSec);
  if (ni->goal_handle && ni->nav_client) {
    ni->nav_client->async_cancel_goal(ni->goal_handle);
    ni->goal_handle.reset();
  }
  ni->has_active_goal = false;
  ni->nav_seq++;
}

void FleetManagerNode::cancel_all_goals()
{
  for (auto & [_, ni] : navs_) cancel_goals(ni);
}

void FleetManagerNode::stop_robot(const std::string & robot_id, int burst)
{
  if (vel_pubs_.find(robot_id) == vel_pubs_.end())
    vel_pubs_[robot_id] = this->create_publisher<geometry_msgs::msg::Twist>(
      "/" + robot_id + "/cmd_vel", 10);
  geometry_msgs::msg::Twist zero;
  for (int i = 0; i < burst; ++i) vel_pubs_[robot_id]->publish(zero);
}

void FleetManagerNode::stop_all()
{
  for (auto & [rid, _] : navs_) stop_robot(rid, 5);
}

std::shared_ptr<RobotNavInfo> FleetManagerNode::get_or_create_nav(
  const std::string & robot_id)
{
  auto it = navs_.find(robot_id);
  if (it != navs_.end()) return it->second;

  auto ni = std::make_shared<RobotNavInfo>();
  ni->nav_client = rclcpp_action::create_client<NavigateToPose>(
    this, "/" + robot_id + "/navigate_to_pose");
  ni->task_cmd_pub = this->create_publisher<fleet_msgs::msg::TaskCmd>(
    "/" + robot_id + "/task/assign", 10);
  ni->task_fb_sub = this->create_subscription<fleet_msgs::msg::TaskFb>(
    "/" + robot_id + "/task/feedback", 10,
    [this, robot_id](const fleet_msgs::msg::TaskFb::SharedPtr m) {
      chassis_fb_callback(robot_id, m);
    });
  ni->led_pub = this->create_publisher<fleet_msgs::msg::LEDTask>(
    "/" + robot_id + "/led/task", 10);
  ni->led_sub = this->create_subscription<fleet_msgs::msg::LEDStatus>(
    "/" + robot_id + "/led/status", rclcpp::QoS(10).reliable(),
    [this, robot_id](const fleet_msgs::msg::LEDStatus::SharedPtr m) {
      led_status_callback(robot_id, m);
    });

  navs_[robot_id] = ni;
  return ni;
}

double FleetManagerNode::normalize_angle(double a) const
{
  while (a >  M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

double FleetManagerNode::get_yaw(const geometry_msgs::msg::Quaternion & q) const
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double r, p, y;
  tf2::Matrix3x3 m(tq);
  m.getRPY(r, p, y);
  return y;
}

std::string FleetManagerNode::join_route(const std::vector<std::string> & wps) const
{
  return join_waypoints(wps);
}

// ==================== Chain Retreat Coordination ====================

bool FleetManagerNode::is_robot_idle(const std::string & robot_id) const
{
  auto ni = navs_.find(robot_id);
  if (ni == navs_.end() || !ni->second) return false;
  return !ni->second->has_active_goal &&
         ni->second->current_task_id.empty() &&
         ni->second->route.empty() &&
         !ni->second->chassis_task_sent;
}

bool FleetManagerNode::is_robot_stationary(const std::string & robot_id) const
{
  auto ni = navs_.find(robot_id);
  if (ni == navs_.end() || !ni->second) return false;
  // Stationary: not physically moving (no active Nav2 goal, no chassis exec)
  return !ni->second->has_active_goal && !ni->second->chassis_task_sent;
}

bool FleetManagerNode::is_mutual_block(
  const std::string & blocker, const std::string & blocker_wp,
  const std::string &, const std::string & requester_wp) const
{
  // Check if blocker has any exit that doesn't go through requester's position
  auto conns = traffic_->get_waypoint_connections(blocker_wp);
  for (const auto & nb : conns) {
    if (nb == requester_wp) continue;  // this path goes through requester
    // Check if nb is free for blocker to enter
    std::string b = occupancy_->can_enter(blocker, blocker_wp, nb);
    if (b.empty() || b == blocker) {
      // Blocker has an independent exit → not a mutual block
      return false;
    }
  }
  // All exits require going through requester → mutual block
  return true;
}

bool FleetManagerNode::try_build_retreat_chain(
  const std::string & requester, const std::string & from_wp,
  const std::string & to_wp, const std::string & blocker,
  const std::set<std::string> & blocked_set, int depth)
{
  if (depth >= kMaxChainDepth) {
    PersistLogger::log_info("nav.chain_depth_exceeded", requester, "",
      "depth=" + std::to_string(depth) + " max=" + std::to_string(kMaxChainDepth),
      __FILE__, __LINE__, __func__);
    return false;
  }

  // Clear current chain steps (will be rebuilt)
  chain_plan_.steps.clear();

  // Find a retreat waypoint for the requester (from from_wp)
  auto conns = traffic_->get_waypoint_connections(from_wp);
  std::string best_retreat;
  std::vector<RetreatChainStep> best_subchain;

  for (const auto & nb : conns) {
    if (nb == to_wp) continue;
    if (blocked_set.count(nb)) continue;

    auto holder = occupancy_->get_zone_holder(nb);
    if (holder.empty() || holder == requester) {
      best_retreat = nb;
      best_subchain.clear();
      break;  // direct free retreat, optimal
    }

    // Occupied — try subchain if holder is stationary
    if (is_robot_stationary(holder)) {
      std::set<std::string> holder_blocked = blocked_set;
      holder_blocked.insert(to_wp);

      // Build subchain in a temporary plan (recursion)
      ChainRetreatPlan saved;
      saved.steps.swap(chain_plan_.steps);

      if (try_build_retreat_chain(holder, nb, from_wp, requester, holder_blocked, depth + 1)) {
        std::vector<RetreatChainStep> candidate = std::move(chain_plan_.steps);
        // Restore for this level's use
        chain_plan_.steps.swap(saved.steps);
        if (best_retreat.empty() || candidate.size() < best_subchain.size()) {
          best_retreat = nb;
          best_subchain = std::move(candidate);
        }
      } else {
        // Failed — restore
        chain_plan_.steps.swap(saved.steps);
      }
    }
  }

  if (best_retreat.empty()) {
    return false;
  }

  // Assemble chain from subchain + current level
  // Subchain steps come first (innermost retreat), then requester's retreat
  for (auto & s : best_subchain) {
    chain_plan_.steps.push_back(std::move(s));
  }

  // Step: Requester retreats to best_retreat
  chain_plan_.steps.push_back({requester, best_retreat});

  // Step: Blocker exits to a free waypoint
  // Blocker will move from to_wp through from_wp to a free waypoint
  std::string blocker_dest;
  for (const auto & nb : traffic_->get_waypoint_connections(from_wp)) {
    if (nb == to_wp || nb == best_retreat) continue;
    if (blocked_set.count(nb)) continue;
    auto b = occupancy_->waypoint_blocker("", nb);
    if (b.empty()) { blocker_dest = nb; break; }
  }
  if (blocker_dest.empty() && !best_retreat.empty()) {
    // Fallback: blocker goes to wherever the requester just retreated from... no!
    // Just find any free waypoint
    blocker_dest = occupancy_->find_nearest_free_waypoint(from_wp, {to_wp, requester});
  }
  if (blocker_dest.empty()) {
    chain_plan_.steps.clear();
    return false;
  }
  chain_plan_.steps.push_back({blocker, blocker_dest});

  // Step: Requester resumes to original target
  chain_plan_.steps.push_back({requester, to_wp});

  return true;
}

void FleetManagerNode::execute_chain_step()
{
  if (!chain_plan_.active || chain_plan_.current_step >= chain_plan_.steps.size()) {
    return;
  }

  const auto & step = chain_plan_.steps[chain_plan_.current_step];
  auto ni = get_or_create_nav(step.robot_id);
  if (!ni) {
    abort_chain("robot nav info missing");
    return;
  }

  // Check robot is still online
  auto st = robots_.find(step.robot_id);
  if (st == robots_.end() || st->second.connection_status != "online") {
    abort_chain("robot offline");
    return;
  }

  // Pre-validation: verify target waypoint is reachable (not held by unexpected robot)
  std::string wp_holder = occupancy_->get_zone_holder(step.target_wp);
  if (!wp_holder.empty() && wp_holder != step.robot_id) {
    // Target waypoint is unexpectedly occupied → abort
    PersistLogger::log_warn("nav.chain_step_blocked", step.robot_id, "",
      "step target " + step.target_wp + " held by " + wp_holder + ", aborting chain",
      __FILE__, __LINE__, __func__);
    abort_chain("step target occupied");
    return;
  }

  // Verify waypoint exists in map
  auto wp_pose = traffic_->get_waypoint_pose(step.target_wp);
  if (wp_pose.position.x == 0.0 && wp_pose.position.y == 0.0 && wp_pose.position.z == 0.0) {
    // Check if this is actually a valid (0,0,0) waypoint or missing
    auto all_wps = traffic_->get_all_waypoint_poses();
    if (all_wps.find(step.target_wp) == all_wps.end()) {
      abort_chain("target waypoint not found: " + step.target_wp);
      return;
    }
  }

  // Reset nav state for chain step
  ni->route = {step.target_wp};
  ni->route_index = 0;
  ni->current_task_id = std::string(kChainTaskPrefix) + step.robot_id;
  ni->retry_count = 0;
  ni->retry_after = rclcpp::Time{};

  PersistLogger::log_info("nav.chain_step", step.robot_id, ni->current_task_id,
    "step " + std::to_string(chain_plan_.current_step + 1) + "/" +
    std::to_string(chain_plan_.steps.size()) + " target=" + step.target_wp,
    __FILE__, __LINE__, __func__);

  navigate_to_waypoint(step.robot_id, step.target_wp, ni->current_task_id, true);
}

void FleetManagerNode::on_chain_step_complete(const std::string & robot_id, bool nav_success)
{
  if (!chain_plan_.active) return;

  const auto & step = chain_plan_.steps[chain_plan_.current_step];

  // Verify this is the expected robot
  if (step.robot_id != robot_id) {
    PersistLogger::log_warn("nav.chain_step_mismatch", robot_id, "",
      "expected " + step.robot_id + " at step " + std::to_string(chain_plan_.current_step),
      __FILE__, __LINE__, __func__);
    return;
  }

  if (!nav_success) {
    chain_plan_.step_retry_count++;
    if (chain_plan_.step_retry_count <= kMaxChainStepRetries) {
      PersistLogger::log_info("nav.chain_step_retry", robot_id, "",
        "retry " + std::to_string(chain_plan_.step_retry_count) + "/" +
        std::to_string(kMaxChainStepRetries),
        __FILE__, __LINE__, __func__);
      execute_chain_step();
      return;
    }
    abort_chain("step nav failed after " + std::to_string(kMaxChainStepRetries) + " retries");
    return;
  }

  // Clear chain robot's nav state after successful step
  auto ni = get_or_create_nav(robot_id);
  if (ni) {
    occupancy_->release_reservations(robot_id);
    ni->current_task_id.clear();
    ni->has_active_goal = false;
    ni->route.clear();
    ni->route_index = 0;
  }

  // Advance to next step
  chain_plan_.step_retry_count = 0;
  chain_plan_.current_step++;

  if (chain_plan_.current_step >= chain_plan_.steps.size()) {
    // Chain complete — restore all saved tasks
    PersistLogger::log_info("nav.chain_complete", chain_plan_.original_requester,
      chain_plan_.original_task_id,
      "chain retreat finished, resuming " + std::to_string(chain_plan_.saved_task_ids.size()) + " tasks",
      __FILE__, __LINE__, __func__);

    // Save and clear chain state before re-starting navigation
    auto saved_tasks = std::move(chain_plan_.saved_task_ids);
    auto saved_targets = std::move(chain_plan_.saved_targets);
    chain_plan_.active = false;
    chain_plan_.steps.clear();
    chain_plan_.current_step = 0;

    // Restore each participant's task
    for (const auto & [rid, tid] : saved_tasks) {
      auto ni = get_or_create_nav(rid);
      if (!ni) continue;
      auto st = robots_.find(rid);
      if (st == robots_.end() || st->second.connection_status != "online") continue;

      // Skip tasks that were cancelled during chain execution
      auto ti_check = scheduler_->get_task_info(tid);
      if (ti_check.task_id.empty() || ti_check.status == "cancelled") {
        PersistLogger::log_info("nav.chain_skip_cancelled", rid, tid,
          "task was cancelled during chain, skipping restore",
          __FILE__, __LINE__, __func__);
        finalize_task_completion(rid, tid);
        continue;
      }

      auto tgt_it = saved_targets.find(rid);
      std::string tgt = (tgt_it != saved_targets.end()) ? tgt_it->second : "";
      if (tgt.empty()) continue;

      // Check if already at target
      auto target_pose = traffic_->get_waypoint_pose(tgt);
      double dx = st->second.current_pose.position.x - target_pose.position.x;
      double dy = st->second.current_pose.position.y - target_pose.position.y;
      if (std::hypot(dx, dy) <= waypoint_radius_) {
        // Already there → complete immediately
        scheduler_->complete_task(tid);
        fleet_msgs::msg::TaskInfo pub_ti = scheduler_->get_task_info(tid);
        if (!pub_ti.task_id.empty()) task_pub_->publish(pub_ti);
        finalize_task_completion(rid, tid);
      } else {
        // Re-navigate
        start_navigation(rid, tgt, tid);
      }
    }
  } else {
    execute_chain_step();
  }
}

void FleetManagerNode::abort_chain(const std::string & reason)
{
  if (!chain_plan_.active) return;

  PersistLogger::log_warn("nav.chain_abort", chain_plan_.original_requester,
    chain_plan_.original_task_id,
    "chain aborted: " + reason + " at step " + std::to_string(chain_plan_.current_step + 1),
    __FILE__, __LINE__, __func__);

  // Clean up all chain robots' nav state
  for (size_t i = chain_plan_.current_step; i < chain_plan_.steps.size(); ++i) {
    const auto & step = chain_plan_.steps[i];
    auto ni = get_or_create_nav(step.robot_id);
    if (ni) {
      cancel_goals(ni);
      occupancy_->release_reservations(step.robot_id);
      stop_robot(step.robot_id, 5);
      ni->current_task_id.clear();
      ni->has_active_goal = false;
      ni->route.clear();
      ni->route_index = 0;
    }
  }

  auto saved_tasks = std::move(chain_plan_.saved_task_ids);
  chain_plan_.active = false;
  chain_plan_.steps.clear();
  chain_plan_.current_step = 0;

  // Re-queue all saved tasks with preserved binding
  for (const auto & [rid, tid] : saved_tasks) {
    if (tid.empty()) continue;
    scheduler_->mark_task_pending_preserve(tid);
    fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
    if (!ti.task_id.empty()) task_pub_->publish(ti);
  }
}

}  // namespace fleet_manager
