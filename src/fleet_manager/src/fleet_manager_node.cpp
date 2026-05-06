#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace fleet_manager
{

// ============================================================================
// 生命周期
// ============================================================================

FleetManagerNode::FleetManagerNode()
: Node("fleet_manager")
{
  // ── 订阅 ──
  fleet_sub_ = this->create_subscription<fleet_msgs::msg::FleetStatus>(
    "/fleet_monitor/fleet_status", 10,
    std::bind(&FleetManagerNode::fleet_status_callback, this, std::placeholders::_1));

  // ── 发布 ──
  traffic_pub_ = this->create_publisher<fleet_msgs::msg::FleetStatus>(
    "/fleet_manager/fleet_status_traffic", 10);
  task_pub_ = this->create_publisher<fleet_msgs::msg::TaskInfo>("~/task_status", 10);
  alert_pub_ = this->create_publisher<std_msgs::msg::String>("~/alerts", 10);
  metrics_pub_ = this->create_publisher<std_msgs::msg::String>("~/metrics", 10);

  // ── 服务 ──
  submit_srv_ = this->create_service<fleet_msgs::srv::SubmitTask>(
    "~/submit_task",
    std::bind(&FleetManagerNode::handle_submit_task, this, std::placeholders::_1, std::placeholders::_2));
  cancel_srv_ = this->create_service<fleet_msgs::srv::CancelTask>(
    "~/cancel_task",
    std::bind(&FleetManagerNode::handle_cancel_task, this, std::placeholders::_1, std::placeholders::_2));
  status_srv_ = this->create_service<fleet_msgs::srv::GetRobotStatus>(
    "~/get_robot_status",
    std::bind(&FleetManagerNode::handle_get_robot_status, this, std::placeholders::_1, std::placeholders::_2));
  load_srv_ = this->create_service<fleet_msgs::srv::LoadTrafficMap>(
    "~/load_traffic_map",
    std::bind(&FleetManagerNode::handle_load_traffic_map, this, std::placeholders::_1, std::placeholders::_2));
  save_srv_ = this->create_service<fleet_msgs::srv::SaveTrafficMap>(
    "~/save_traffic_map",
    std::bind(&FleetManagerNode::handle_save_traffic_map, this, std::placeholders::_1, std::placeholders::_2));
  remove_srv_ = this->create_service<fleet_msgs::srv::RemoveRobot>(
    "~/remove_robot",
    std::bind(&FleetManagerNode::handle_remove_robot, this, std::placeholders::_1, std::placeholders::_2));

  // ── 核心子模块 ──
  scheduler_  = std::make_unique<TaskScheduler>(this);
  traffic_    = std::make_unique<TrafficManager>(this);
  occupancy_  = std::make_unique<OccupancyManager>(this);

  // ── 参数声明 ──
  this->declare_parameter("traffic_map_file", "");
  this->declare_parameter("waypoint_acceptance_radius", 0.5);
  this->declare_parameter("traffic_segment_lateral_max", 1.2);
  this->declare_parameter("persist_log_enabled", true);
  this->declare_parameter("persist_log_dir", "test_logs");
  this->declare_parameter("persist_log_verbose_info", false);
  this->declare_parameter("scheduler_interval_sec", 1.0);
  this->declare_parameter("nav_retry_base_sec", 1.0);
  this->declare_parameter("nav_retry_max", 5);
  this->declare_parameter("nav_stuck_timeout_sec", 20.0);
  this->declare_parameter("nav_absolute_timeout_sec", 45.0);
  this->declare_parameter("chassis_handshake_timeout_sec", 5.0);
  this->declare_parameter("chassis_exec_timeout_sec", 30.0);
  this->declare_parameter("chassis_max_retries", 3);
  this->declare_parameter("monitor_fleet_stale_timeout_sec", 4.0);
  this->declare_parameter("ghost_lock_ttl_sec", 120.0);
  this->declare_parameter("deadlock_timeout_sec", 10.0);
  this->declare_parameter("max_task_retry_cycles", 5);

  // ── 参数读取(含安全下限) ──
  waypoint_radius_  = this->get_parameter("waypoint_acceptance_radius").as_double();
  segment_lateral_  = this->get_parameter("traffic_segment_lateral_max").as_double();
  sched_interval_   = std::max(0.3, this->get_parameter("scheduler_interval_sec").as_double());
  retry_base_       = std::max(0.5, this->get_parameter("nav_retry_base_sec").as_double());
  retry_max_        = std::max(1, static_cast<int>(this->get_parameter("nav_retry_max").as_int()));
  nav_stuck_timeout_    = this->get_parameter("nav_stuck_timeout_sec").as_double();
  nav_absolute_timeout_  = this->get_parameter("nav_absolute_timeout_sec").as_double();
  chassis_hs_timeout_   = this->get_parameter("chassis_handshake_timeout_sec").as_double();
  chassis_exec_timeout_ = this->get_parameter("chassis_exec_timeout_sec").as_double();
  chassis_max_retries_  = this->get_parameter("chassis_max_retries").as_int();
  monitor_stale_timeout_ = this->get_parameter("monitor_fleet_stale_timeout_sec").as_double();
  ghost_lock_ttl_   = std::max(10.0, this->get_parameter("ghost_lock_ttl_sec").as_double());
  deadlock_timeout_ = std::max(5.0, this->get_parameter("deadlock_timeout_sec").as_double());
  max_task_retry_cycles_ = std::max(2, static_cast<int>(this->get_parameter("max_task_retry_cycles").as_int()));

  // ── 持久化日志初始化 ──
  {
    bool en = this->get_parameter("persist_log_enabled").as_bool();
    std::string dir = this->get_parameter("persist_log_dir").as_string();
    bool vb = this->get_parameter("persist_log_verbose_info").as_bool();
    PersistLogger::init(en, dir, "fleet_manager", vb);
  }

  // ── 加载交通图并注入拓扑到占用管理器 ──
  std::string map_file = this->get_parameter("traffic_map_file").as_string();
  if (!map_file.empty() && traffic_->load_map(map_file)) {
    RCLCPP_INFO(this->get_logger(), "Loaded traffic map: %s", map_file.c_str());
    occupancy_->set_topology(
      traffic_->get_adjacency_map(),
      [this](const std::string & id) { return traffic_->get_waypoint_pose(id); },
      [this](const std::string & id) { return traffic_->get_waypoint_radius(id); });
    traffic_->validate_waypoint_spacing(waypoint_radius_ * 2.0);
  }

  // ── 定时器: 主循环 500ms + 快循环 200ms ──
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&FleetManagerNode::control_timer_callback, this));
  fast_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&FleetManagerNode::fast_timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Fleet Manager started (v2 zone-based)");
}

FleetManagerNode::~FleetManagerNode()
{
  if (control_timer_) control_timer_->cancel();
  if (fast_timer_)    fast_timer_->cancel();
  stop_all();
  cancel_all_goals();
}

// ============================================================================
// 主循环 (500ms)
// ============================================================================

void FleetManagerNode::control_timer_callback()
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  ++tick_;

  // 清理过期的幽灵锁
  occupancy_->expire_ghost_locks(this->now(), ghost_lock_ttl_);

  // 更新所有在线底盘的位置并同步 zone_locks
  for (auto & [rid, st] : robots_) {
    if (st.connection_status != "online") continue;
    if (std::abs(st.current_pose.position.x) < 1e-6 &&
        std::abs(st.current_pose.position.y) < 1e-6) {
      // 零位姿 = 定位丢失 → 释放锁防止幽灵阻塞
      occupancy_->release_locks(rid);
      occupancy_->release_reservations(rid);
      st.location_type = "unknown";
      st.current_waypoint = "";
      st.current_segment = "";
      continue;
    }

    auto prev_loc = occupancy_->get_location(rid);
    auto loc = occupancy_->update_location(
      rid, st.current_pose, waypoint_radius_, segment_lateral_);
    if (loc.type != prev_loc.type ||
        loc.waypoint_id != prev_loc.waypoint_id ||
        loc.segment_from != prev_loc.segment_from ||
        loc.segment_to != prev_loc.segment_to) {
      wake_waiters(rid);
    }

    if (loc.type == LocationType::WAYPOINT) {
      st.location_type = "waypoint";
      st.current_waypoint = loc.waypoint_id;
      st.current_segment = "";
    } else if (loc.type == LocationType::SEGMENT) {
      st.location_type = "segment";
      st.current_waypoint = "";
      st.current_segment = loc.segment_from + "->" + loc.segment_to;
    } else {
      st.location_type = "unknown";
    }
  }

  check_arrivals();
  chassis_timeout_check();

  // 按 sched_interval_ 分频调度
  static int sched_phase = 0;
  int period = std::max(1, static_cast<int>(sched_interval_ / 0.5));
  if (++sched_phase >= period) {
    sched_phase = 0;
    schedule_tick();
  }

  deadlock_check();

  // metrics 每 5s 发布一次
  const auto now = this->now();
  if (last_metrics_time_.nanoseconds() == 0 ||
      (now - last_metrics_time_).seconds() >= 5.0) {
    last_metrics_time_ = now;
    publish_metrics();
  }

  publish_traffic_fleet_status();
}

// ============================================================================
// 快循环 (200ms) — LED 状态推送 + 导航重试
// ============================================================================

void FleetManagerNode::fast_timer_callback()
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  led_timer_callback();

  // 重试被阻塞的导航(退避窗口已过的机器人)
  for (auto & [rid, ni] : navs_) {
    if (!ni || ni->has_active_goal) continue;
    if (ni->current_task_id.empty()) continue;
    if (ni->route.empty()) continue;
    if (ni->retry_after.nanoseconds() > 0 && this->now() < ni->retry_after) continue;

    navigate_to_next_waypoint(rid);
  }
}

// ============================================================================
// 车队状态回调 — fleet_monitor 数据到达时更新底盘集合
// ============================================================================

void FleetManagerNode::fleet_status_callback(
  const fleet_msgs::msg::FleetStatus::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  last_fleet_ = *msg;
  has_fleet_ = true;
  last_fleet_time_ = this->now();

  // 记录之前在线的底盘
  std::set<std::string> prev_online;
  for (const auto & [rid, st] : robots_)
    if (st.connection_status == "online") prev_online.insert(rid);

  // 更新已发现底盘的状态
  std::set<std::string> reported;
  for (auto & r : msg->robots) {
    if (removed_.count(r.robot_id)) {
      if (r.connection_status == "online") removed_.erase(r.robot_id);
      else continue;
    }
    reported.insert(r.robot_id);
    robots_[r.robot_id] = r;
    get_or_create_nav(r.robot_id);
  }

  // 未在本轮上报的底盘标记为 offline
  for (auto & [rid, st] : robots_) {
    if (reported.count(rid)) continue;
    if (st.connection_status == "online") st.connection_status = "offline";
  }

  // 在线→离线: 释放预约, 保留 zone_locks 作为幽灵守卫, 启动幽灵 TTL
  for (const auto & rid : prev_online) {
    auto it = robots_.find(rid);
    if (it != robots_.end() && it->second.connection_status != "online") {
      occupancy_->release_reservations(rid);
      occupancy_->mark_ghost(rid, this->now());
      auto ni = get_or_create_nav(rid);
      if (ni) { cancel_goals(ni); stop_robot(rid, 10); }

      // 清理事件等待: 唤醒等待离线机器人的请求者，清除离线者的等待条目
      waiting_for_.erase(rid);
      for (auto & [_, waiters] : waiting_for_) waiters.erase(rid);

      // 如果离线底盘是活跃链的参与者，立即中止链
      if (chain_plan_.active) {
        bool is_participant = false;
        for (const auto & s : chain_plan_.steps) {
          if (s.robot_id == rid) { is_participant = true; break; }
        }
        if (!is_participant && chain_plan_.saved_task_ids.count(rid))
          is_participant = true;
        if (is_participant) {
          PersistLogger::log_warn("chain.participant_offline", rid, "",
            "aborting chain due to participant offline",
            __FILE__, __LINE__, __func__);
          abort_chain("participant offline: " + rid);
        }
      }

      PersistLogger::log_warn("robot.offline", rid, "",
        "positions locked as ghost guard (TTL=" + std::to_string(ghost_lock_ttl_) + "s)",
        __FILE__, __LINE__, __func__);
    }
  }

  // 离线→在线: 清除幽灵锁, 让 update_location 重新建立 zone_locks
  for (const auto & [rid, st] : robots_) {
    auto prev = prev_online.find(rid);
    if (prev == prev_online.end() && st.connection_status == "online") {
      occupancy_->release_locks(rid);
      occupancy_->release_reservations(rid);
      occupancy_->clear_ghost(rid);
      auto ni = get_or_create_nav(rid);
      if (ni) {
        ni->chassis_task_sent = false;
        ni->chassis_handshake_ok = false;
        ni->chassis_retries = 0;
      }
      PersistLogger::log_info("robot.online", rid, "",
        "back online, ghost locks cleared", __FILE__, __LINE__, __func__);
    }
  }
}

// ============================================================================
// 调度主循环
// ============================================================================

void FleetManagerNode::schedule_tick()
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  occupancy_->expire_stale_reservations();
  scheduler_->purge_finished(200);

  assign_pending_tasks();
  if (chain_plan_.active) return;

  // ── 孤儿任务恢复（在 assign 之后，确保链参与者已被保护）──
  const auto now = this->now();
  for (const auto & t : scheduler_->get_all_tasks()) {
    if (t.status != "in_progress" && t.status != "waiting_fleet" &&
        t.status != "assigned" && t.status != "executing") continue;
    if (t.assigned_robot_id.empty()) continue;

    const std::string & rid = t.assigned_robot_id;

    // 跳过活跃链参与者，不干扰链执行
    if (chain_plan_.active) {
      bool is_participant = false;
      for (const auto & s : chain_plan_.steps)
        if (s.robot_id == rid) { is_participant = true; break; }
      if (!is_participant)
        is_participant = chain_plan_.saved_task_ids.count(rid) > 0;
      if (is_participant) continue;
    }

    auto ni = get_or_create_nav(rid);
    auto rit = robots_.find(rid);
    bool offline = (rit == robots_.end() || rit->second.connection_status != "online");
    bool nav_idle = ni && !ni->has_active_goal && ni->route.empty();
    bool nav_bound = ni && ni->current_task_id == t.task_id;
    double age_sec = (now - t.started_at).seconds();

    if (t.status == "executing") {
      // 底盘执行中超时由 chassis_timeout_check 处理；仅离线过久才失败
      if (offline && age_sec > chassis_hs_timeout_ + chassis_exec_timeout_ + 10.0) {
        scheduler_->fail_task(t.task_id, "robot offline during execution");
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
        if (!ti.task_id.empty()) task_pub_->publish(ti);
        if (ni) finalize_task_completion(rid, t.task_id);
      }
      continue;
    }

    if (t.status == "in_progress") {
      if (nav_bound && nav_idle && !ni->chassis_task_sent) {
        PersistLogger::log_warn("sched.orphan_in_progress", rid, t.task_id,
          "in_progress task with idle nav, recovering", __FILE__, __LINE__, __func__);
        scheduler_->mark_task_pending(t.task_id);
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
        if (!ti.task_id.empty()) task_pub_->publish(ti);
      }
      continue;
    }

    if (t.status == "waiting_fleet") {
      if (offline && nav_idle && age_sec > 30.0) {
        PersistLogger::log_warn("sched.orphan_waiting", rid, t.task_id,
          "waiting_fleet task on offline robot for " + std::to_string(age_sec) + "s, recovering",
          __FILE__, __LINE__, __func__);
        scheduler_->mark_task_pending(t.task_id);
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
        if (!ti.task_id.empty()) task_pub_->publish(ti);
      }
      continue;
    }

    if (t.status == "assigned") {
      if (nav_bound && nav_idle && age_sec > 10.0) {
        PersistLogger::log_warn("sched.orphan_assigned", rid, t.task_id,
          "assigned task idle for " + std::to_string(age_sec) + "s, recovering",
          __FILE__, __LINE__, __func__);
        scheduler_->mark_task_pending(t.task_id);
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
        if (!ti.task_id.empty()) task_pub_->publish(ti);
      }
      continue;
    }
  }

  // (assign_pending_tasks already called above)
}

// ============================================================================
// 任务分配
// ============================================================================

void FleetManagerNode::assign_pending_tasks()
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  const auto now = this->now();
  if (!occupancy_->get_conflict_hubs().empty() ||
      !occupancy_->get_conflict_edges().empty()) {
    PersistLogger::log_warn("sched.resource_conflict_pause", "", "",
      "resource conflict active, pausing new assignments",
      __FILE__, __LINE__, __func__);
    return;
  }
  if (chain_plan_.active) {
    PersistLogger::log_info("sched.chain_pause", chain_plan_.original_requester,
      chain_plan_.original_task_id,
      "chain retreat active, pausing normal assignments",
      __FILE__, __LINE__, __func__);
    return;
  }

  // 检查 fleet_monitor 数据是否陈旧
  bool monitor_stale =
    has_fleet_ && last_fleet_time_.nanoseconds() > 0 &&
    monitor_stale_timeout_ > 0.0 &&
    (now - last_fleet_time_).seconds() > monitor_stale_timeout_;

  if (monitor_stale) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "fleet monitor stale, pausing scheduling");
    return;
  }

  scheduler_->repair_queue();
  auto has_active_waiters = [&](const std::string & blocker_id) {
    for (const auto & [_, wait] : task_waits_) {
      if (wait.blocker_id == blocker_id) return true;
    }
    auto it = waiting_for_.find(blocker_id);
    if (it == waiting_for_.end()) return false;
    for (const auto & waiter_id : it->second) {
      auto wi = navs_.find(waiter_id);
      if (wi != navs_.end() && wi->second &&
          !wi->second->current_task_id.empty() &&
          !wi->second->route.empty()) {
        return true;
      }
    }
    return false;
  };
  auto target_has_active_task = [&](const std::string & target_wp,
      const std::string & current_task_id) {
    for (const auto & existing : scheduler_->get_all_tasks()) {
      if (existing.task_id.empty() || existing.task_id == current_task_id) continue;
      if (existing.waypoint_id != target_wp) continue;
      if (existing.status == "completed" || existing.status == "failed" ||
          existing.status == "cancelled" || existing.status == "pending") continue;
      return existing.task_id;
    }
    return std::string{};
  };
  auto active_task_targets = [&](const std::string & current_task_id) {
    std::set<std::string> targets;
    for (const auto & existing : scheduler_->get_all_tasks()) {
      if (existing.task_id.empty() || existing.task_id == current_task_id) continue;
      if (existing.waypoint_id.empty()) continue;
      if (existing.status == "completed" || existing.status == "failed" ||
          existing.status == "cancelled" || existing.status == "pending") {
        continue;
      }
      targets.insert(existing.waypoint_id);
    }
    return targets;
  };
  auto wait_condition_ready = [&](const fleet_msgs::msg::TaskInfo & t) {
    auto it = task_waits_.find(t.task_id);
    if (it == task_waits_.end()) return true;
    const auto & w = it->second;
    bool ready = false;
    if (w.state == TaskWaitState::WAIT_TARGET_CLEAR) {
      ready = occupancy_->waypoint_blocker(w.robot_id, w.target_wp).empty();
    } else if (w.state == TaskWaitState::WAIT_BLOCKER_RELEASE ||
               w.state == TaskWaitState::SELF_RELOCATING) {
      if (w.from_wp.empty() || w.to_wp.empty()) ready = true;
      else {
        auto blocker = occupancy_->can_enter(w.robot_id, w.from_wp, w.to_wp);
        ready = blocker.empty() || blocker != w.blocker_id;
      }
    } else {
      ready = true;
    }
    if (ready) {
      PersistLogger::log_info("sched.wait_condition_ready", w.robot_id, w.task_id,
        "state ready for task dispatch",
        __FILE__, __LINE__, __func__);
      task_waits_.erase(it);
    }
    return ready;
  };
  auto progress_wait_conditions = [&]() {
    for (auto & [task_id, wait] : task_waits_) {
      if (wait.state == TaskWaitState::WAIT_TARGET_CLEAR) {
        if (wait.target_wp.empty()) continue;
        auto target_holder = occupancy_->get_zone_holder(wait.target_wp);
        if (target_holder.empty() || target_holder == wait.robot_id) continue;
        wait.blocker_id = target_holder;

        auto waiter_ni = get_or_create_nav(wait.robot_id);
        if (waiter_ni && (!waiter_ni->current_task_id.empty() ||
            waiter_ni->has_active_goal || !waiter_ni->route.empty())) {
          continue;
        }
        if (!is_robot_idle(target_holder)) continue;

        std::set<std::string> final_exclude;
        std::set<std::string> transit_exclude;
        std::string waiter_wp;
        auto ws = robots_.find(wait.robot_id);
        if (ws != robots_.end()) {
          waiter_wp = ws->second.current_waypoint;
          if (waiter_wp.empty()) waiter_wp = traffic_->find_nearest_waypoint(ws->second.current_pose);
          if (!waiter_wp.empty()) {
            auto path = traffic_->find_path(waiter_wp, wait.target_wp);
            for (const auto & wp : path) {
              final_exclude.insert(wp);
            }
          }
        }
        final_exclude.insert(wait.target_wp);
        for (const auto & active_target : active_task_targets(task_id)) {
          final_exclude.insert(active_target);
        }

        std::vector<std::string> exit_path;
        double best_cost = std::numeric_limits<double>::max();
        auto all_wps = traffic_->get_all_waypoint_poses();
        auto from_pose = traffic_->get_waypoint_pose(wait.target_wp);
        for (const auto & [candidate, pose] : all_wps) {
          if (candidate == wait.target_wp || final_exclude.count(candidate)) continue;
          if (!occupancy_->is_zone_free_for(target_holder, candidate)) continue;

          auto path = traffic_->find_path(wait.target_wp, candidate);
          if (path.size() < 2) continue;

          bool blocked_path = false;
          for (size_t i = 1; i < path.size(); ++i) {
            if (path[i] != candidate && transit_exclude.count(path[i])) {
              blocked_path = true;
              break;
            }
            auto hop_blocker = occupancy_->can_enter(target_holder, path[i - 1], path[i]);
            if (!hop_blocker.empty() && hop_blocker != target_holder) {
              if (hop_blocker == wait.robot_id) {
                blocked_path = true;
                break;
              }
            }
          }
          if (blocked_path) continue;

          const double dist = std::hypot(
            pose.position.x - from_pose.position.x,
            pose.position.y - from_pose.position.y);
          const double cost = static_cast<double>(path.size()) * 10.0 + dist;
          if (cost < best_cost) {
            best_cost = cost;
            exit_path = path;
          }
        }

        if (exit_path.size() < 2) {
          if (!waiter_wp.empty() && wait.retreat_count < 3) {
            std::vector<std::string> relocate_path;
            double best_relocate_cost = std::numeric_limits<double>::max();
            auto waiter_pose = traffic_->get_waypoint_pose(waiter_wp);
            for (const auto & [candidate, pose] : all_wps) {
              if (candidate == waiter_wp || candidate == wait.target_wp ||
                  final_exclude.count(candidate)) {
                continue;
              }
              if (!occupancy_->is_zone_free_for(wait.robot_id, candidate)) continue;

              auto path = traffic_->find_path(waiter_wp, candidate);
              if (path.size() < 2) continue;

              bool blocked_path = false;
              for (size_t i = 1; i < path.size(); ++i) {
                if (path[i] == wait.target_wp) {
                  blocked_path = true;
                  break;
                }
              }
              if (blocked_path) continue;

              bool opens_holder_exit = false;
              for (const auto & [holder_candidate, holder_pose] : all_wps) {
                (void)holder_pose;
                if (holder_candidate == wait.target_wp ||
                    holder_candidate == candidate ||
                    final_exclude.count(holder_candidate)) {
                  continue;
                }
                if (!occupancy_->is_zone_free_for(target_holder, holder_candidate)) continue;

                auto holder_path = traffic_->find_path(wait.target_wp, holder_candidate);
                if (holder_path.size() < 2) continue;

                bool holder_blocked = false;
                for (size_t j = 1; j < holder_path.size(); ++j) {
                  if (holder_path[j - 1] == candidate || holder_path[j] == candidate) {
                    holder_blocked = true;
                    break;
                  }
                  if (holder_path[j] != holder_candidate && transit_exclude.count(holder_path[j])) {
                    holder_blocked = true;
                    break;
                  }
                }
                if (!holder_blocked) {
                  opens_holder_exit = true;
                  break;
                }
              }
              if (!opens_holder_exit) continue;

              const double dist = std::hypot(
                pose.position.x - waiter_pose.position.x,
                pose.position.y - waiter_pose.position.y);
              const double cost = static_cast<double>(path.size()) * 10.0 + dist;
              if (cost < best_relocate_cost) {
                best_relocate_cost = cost;
                relocate_path = path;
              }
            }

            if (relocate_path.size() >= 2) {
              waiter_ni->route = relocate_path;
              waiter_ni->route_index = 0;
              waiter_ni->current_task_id = "avoidance_" + wait.robot_id;
              waiter_ni->retry_count = 0;
              waiter_ni->retry_after = rclcpp::Time{};
              wait.retreat_count++;
              PersistLogger::log_info("sched.target_clear_requester_relocate", wait.robot_id, task_id,
                "moving requester from " + waiter_wp + " to " + relocate_path.back() +
                " to open exit for target holder " + target_holder,
                __FILE__, __LINE__, __func__);
              navigate_to_next_waypoint(wait.robot_id);
              continue;
            }
          }

          PersistLogger::log_warn("sched.target_clear_exit_unavailable", target_holder, task_id,
            "no exit for target holder at " + wait.target_wp +
            " for requester " + wait.robot_id,
            __FILE__, __LINE__, __func__);
          continue;
        }

        auto holder_ni = get_or_create_nav(target_holder);
        if (!holder_ni) continue;
        holder_ni->route = exit_path;
        holder_ni->route_index = 0;
        holder_ni->current_task_id = "avoidance_" + target_holder;
        holder_ni->retry_count = 0;
        holder_ni->retry_after = rclcpp::Time{};
        waiting_for_[target_holder].insert(wait.robot_id);
        PersistLogger::log_info("sched.target_clear_exit_reserved", target_holder, task_id,
          "moving target holder from " + wait.target_wp + " to " + exit_path.back() +
          " for requester " + wait.robot_id,
          __FILE__, __LINE__, __func__);
        navigate_to_next_waypoint(target_holder);
        continue;
      }

      if (wait.state != TaskWaitState::SELF_RELOCATING &&
          wait.state != TaskWaitState::WAIT_BLOCKER_RELEASE) {
        continue;
      }
      if (wait.blocker_id.empty() || wait.from_wp.empty() || wait.to_wp.empty()) continue;

      auto blocker = occupancy_->can_enter(wait.robot_id, wait.from_wp, wait.to_wp);
      if (blocker.empty() || blocker != wait.blocker_id) continue;

      auto waiter_ni = get_or_create_nav(wait.robot_id);
      if (waiter_ni && (!waiter_ni->current_task_id.empty() ||
          waiter_ni->has_active_goal || !waiter_ni->route.empty())) {
        continue;
      }
      if (!is_robot_idle(wait.blocker_id)) continue;

      std::string blocker_wp = wait.to_wp;
      auto holder = occupancy_->get_zone_holder(wait.to_wp);
      if (holder != wait.blocker_id) {
        auto bs = robots_.find(wait.blocker_id);
        if (bs == robots_.end()) continue;
        blocker_wp = bs->second.current_waypoint;
        if (blocker_wp.empty()) blocker_wp = traffic_->find_nearest_waypoint(bs->second.current_pose);
      }
      if (blocker_wp.empty()) continue;

      std::set<std::string> final_exclude;
      std::set<std::string> transit_exclude;
      auto ws = robots_.find(wait.robot_id);
      if (ws != robots_.end() && !wait.target_wp.empty()) {
        std::string waiter_wp = ws->second.current_waypoint;
        if (waiter_wp.empty()) waiter_wp = traffic_->find_nearest_waypoint(ws->second.current_pose);
        if (!waiter_wp.empty()) {
          auto path = traffic_->find_path(waiter_wp, wait.target_wp);
          for (const auto & wp : path) {
            final_exclude.insert(wp);
            if (wp != wait.from_wp && wp != blocker_wp) transit_exclude.insert(wp);
          }
        }
      }
      final_exclude.insert(wait.from_wp);
      final_exclude.insert(wait.to_wp);
      if (!wait.target_wp.empty()) {
        final_exclude.insert(wait.target_wp);
        transit_exclude.insert(wait.target_wp);
      }

      std::vector<std::string> exit_path;
      double best_cost = std::numeric_limits<double>::max();
      auto all_wps = traffic_->get_all_waypoint_poses();
      auto from_pose = traffic_->get_waypoint_pose(blocker_wp);
      for (const auto & [candidate, pose] : all_wps) {
        if (candidate == blocker_wp || final_exclude.count(candidate)) continue;
        if (!occupancy_->is_zone_free_for(wait.blocker_id, candidate)) continue;

        auto path = traffic_->find_path(blocker_wp, candidate);
        if (path.size() < 2) continue;

        bool blocked_path = false;
        for (size_t i = 1; i < path.size(); ++i) {
          if (path[i] != candidate && transit_exclude.count(path[i])) {
            blocked_path = true;
            break;
          }
          auto hop_blocker = occupancy_->can_enter(wait.blocker_id, path[i - 1], path[i]);
          if (!hop_blocker.empty() && hop_blocker != wait.blocker_id) {
            blocked_path = true;
            break;
          }
        }
        if (blocked_path) continue;

        const double dist = std::hypot(
          pose.position.x - from_pose.position.x,
          pose.position.y - from_pose.position.y);
        const double cost = static_cast<double>(path.size()) * 10.0 + dist;
        if (cost < best_cost) {
          best_cost = cost;
          exit_path = path;
        }
      }

      if (exit_path.size() < 2) {
        PersistLogger::log_warn("sched.blocker_exit_unavailable", wait.blocker_id, task_id,
          "no exit for blocker at " + blocker_wp + " while " + wait.robot_id +
          " waits for " + wait.from_wp + "->" + wait.to_wp,
          __FILE__, __LINE__, __func__);
        continue;
      }

      auto blocker_ni = get_or_create_nav(wait.blocker_id);
      if (!blocker_ni) continue;
      blocker_ni->route = exit_path;
      blocker_ni->route_index = 0;
      blocker_ni->current_task_id = "avoidance_" + wait.blocker_id;
      blocker_ni->retry_count = 0;
      blocker_ni->retry_after = rclcpp::Time{};
      wait.state = TaskWaitState::WAIT_BLOCKER_RELEASE;
      PersistLogger::log_info("sched.blocker_exit_reserved", wait.blocker_id, task_id,
        "moving blocker from " + blocker_wp + " to " + exit_path.back() +
        " for waiter " + wait.robot_id,
        __FILE__, __LINE__, __func__);
      navigate_to_next_waypoint(wait.blocker_id);
    }
  };
  progress_wait_conditions();

  // ── waiting_fleet 重调度: 退避期满的 deferred 任务恢复执行 ──
  for (const auto & t : scheduler_->get_all_tasks()) {
    if (t.status != "waiting_fleet") continue;
    if (t.assigned_robot_id.empty()) continue;

    const std::string & rid = t.assigned_robot_id;
    auto ni = get_or_create_nav(rid);
    if (!ni) continue;

    // 跳过已被其他任务占用的底盘
    if (!ni->current_task_id.empty() && ni->current_task_id != t.task_id) continue;
    if (ni->has_active_goal || !ni->route.empty() || ni->chassis_task_sent) continue;
    if (!wait_condition_ready(t)) continue;
    if (ni->retry_after.nanoseconds() > 0 && now < ni->retry_after) continue;

    std::string active_target_task = target_has_active_task(t.waypoint_id, t.task_id);
    if (!active_target_task.empty()) {
      PersistLogger::log_info("sched.target_active_defer", rid, t.task_id,
        "target " + t.waypoint_id + " already has active task " + active_target_task,
        __FILE__, __LINE__, __func__);
      continue;
    }

    std::string target_blocker = occupancy_->waypoint_blocker(rid, t.waypoint_id);
    if (!target_blocker.empty()) {
      waiting_for_[target_blocker].insert(rid);
      task_waits_[t.task_id] = {TaskWaitState::WAIT_TARGET_CLEAR, rid,
        t.task_id, target_blocker, std::string{}, std::string{}, t.waypoint_id, 0};
      PersistLogger::log_info("sched.target_blocked_defer", rid, t.task_id,
        "target " + t.waypoint_id + " blocked by " + target_blocker,
        __FILE__, __LINE__, __func__);
      continue;
    }

    auto st = robots_.find(rid);
    if (st == robots_.end() || st->second.connection_status != "online") continue;
    if (has_active_waiters(rid)) {
      PersistLogger::log_info("sched.blocker_priority_hold", rid, t.task_id,
        "robot is blocking active waiters, delaying deferred task",
        __FILE__, __LINE__, __func__);
      continue;
    }

    // 重试周期耗尽 → 失败任务，防止无限循环
    if (scheduler_->would_exceed_retry_cycles(t.task_id, max_task_retry_cycles_)) {
      PersistLogger::log_error("sched.redispatch_exhausted", rid, t.task_id,
        "max retry cycles exceeded for waiting_fleet task, failing",
        __FILE__, __LINE__, __func__);
      scheduler_->fail_task(t.task_id, "max retry cycles exceeded");
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
      if (!ti.task_id.empty()) task_pub_->publish(ti);
      occupancy_->release_reservations(rid);
      ni->current_task_id.clear();
      ni->route.clear();
      ni->route_index = 0;
      ni->retry_count = 0;
      continue;
    }

    PersistLogger::log_info("sched.redispatch_waiting", rid, t.task_id,
      "retrying deferred task to wp=" + t.waypoint_id,
      __FILE__, __LINE__, __func__);
    start_navigation(rid, t.waypoint_id, t.task_id);
  }

  // ── 构建可用底盘列表 ──
  std::vector<fleet_msgs::msg::RobotStatus> online;
  for (const auto & [rid, st] : robots_) {
    auto ni = get_or_create_nav(rid);
    if (!ni) continue;

    if (st.connection_status != "online") continue;
    if (std::abs(st.current_pose.position.x) < 1e-6 &&
        std::abs(st.current_pose.position.y) < 1e-6) continue;

    // 跳过被占用或冷却期的底盘
    if (ni->has_active_goal || !ni->route.empty() ||
        !ni->current_task_id.empty() || ni->chassis_task_sent) continue;
    if (ni->retry_after.nanoseconds() > 0 && now < ni->retry_after) continue;
    if (has_active_waiters(rid)) continue;

    auto st_copy = st;
    st_copy.connection_status = "online";
    online.push_back(st_copy);
  }

  if (online.empty()) return;

  auto wp_poses = traffic_->get_all_waypoint_poses();
  auto assigned = scheduler_->assign_tasks_batch(online, wp_poses);

  auto defer_assignment = [&](fleet_msgs::msg::TaskInfo & t,
      const std::string & tag, const std::string & reason, double retry_sec) {
    if (t.task_id.empty()) return;
    PersistLogger::log_warn(tag, t.assigned_robot_id, t.task_id,
      reason, __FILE__, __LINE__, __func__);
    scheduler_->mark_task_pending(t.task_id);
    auto defer_ni = get_or_create_nav(t.assigned_robot_id);
    if (defer_ni) {
      defer_ni->retry_after = now + rclcpp::Duration::from_seconds(retry_sec);
    }
    fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
    if (!ti.task_id.empty()) task_pub_->publish(ti);
    t.task_id.clear();
  };

  std::map<std::string, std::string> claimed_targets;
  std::map<std::string, std::string> claimed_hubs;
  for (auto & t : assigned) {
    if (t.task_id.empty()) continue;

    auto target_claim = claimed_targets.find(t.waypoint_id);
    if (target_claim != claimed_targets.end()) {
      defer_assignment(t, "sched.target_gate_defer",
        "target " + t.waypoint_id + " already claimed by " + target_claim->second,
        3.0);
      continue;
    }

    std::string swp;
    auto st = robots_.find(t.assigned_robot_id);
    if (st != robots_.end()) {
      swp = st->second.current_waypoint;
      if (swp.empty()) swp = traffic_->find_nearest_waypoint(st->second.current_pose);
    }

    std::vector<std::string> path;
    if (!swp.empty()) path = traffic_->find_path(swp, t.waypoint_id);
    if (path.empty() && !swp.empty()) path = {swp, t.waypoint_id};
    if (path.empty()) path = {t.waypoint_id};

    std::string blocked_hub;
    std::string blocked_by;
    for (const auto & wp : path) {
      if (traffic_->get_waypoint_connections(wp).size() < 3) continue;
      auto hub_claim = claimed_hubs.find(wp);
      if (hub_claim != claimed_hubs.end()) {
        blocked_hub = wp;
        blocked_by = hub_claim->second;
        break;
      }
    }
    if (!blocked_hub.empty()) {
      defer_assignment(t, "sched.hub_gate_defer",
        "critical hub " + blocked_hub + " already claimed by " + blocked_by,
        3.0);
      continue;
    }

    claimed_targets[t.waypoint_id] = t.task_id;
    for (const auto & wp : path) {
      if (traffic_->get_waypoint_connections(wp).size() >= 3)
        claimed_hubs[wp] = t.task_id;
    }
  }

  // ── 瓶颈冲突检测: 检测对向路径冲突，defer 低优先级任务 ──
  if (assigned.size() >= 2) {
    std::map<std::string, std::vector<std::string>> paths;
    std::map<std::string, std::set<std::pair<std::string, std::string>>> edges_used;
    for (const auto & t : assigned) {
      if (t.task_id.empty()) continue;
      auto st = robots_.find(t.assigned_robot_id);
      if (st == robots_.end()) continue;

      std::string swp = st->second.current_waypoint;
      if (swp.empty()) swp = traffic_->find_nearest_waypoint(st->second.current_pose);
      if (swp.empty()) continue;

      auto p = traffic_->find_path(swp, t.waypoint_id);
      if (p.size() < 2) continue;
      paths[t.task_id] = p;

      std::set<std::pair<std::string, std::string>> edges;
      for (size_t i = 0; i + 1 < p.size(); ++i)
        edges.insert({p[i], p[i + 1]});
      edges_used[t.task_id] = std::move(edges);
    }

    for (size_t i = 0; i < assigned.size(); ++i) {
      if (assigned[i].task_id.empty()) continue;
      auto ei = edges_used.find(assigned[i].task_id);
      if (ei == edges_used.end()) continue;

      for (size_t j = i + 1; j < assigned.size(); ++j) {
        if (assigned[j].task_id.empty()) continue;
        auto ej = edges_used.find(assigned[j].task_id);
        if (ej == edges_used.end()) continue;

        bool conflict = false;
        for (const auto & [a, b] : ei->second) {
          if (ej->second.count({b, a})) { conflict = true; break; }
        }
        if (!conflict) continue;

        // 对向冲突: defer 低优先级的一方
        std::string defer_tid = (assigned[i].priority < assigned[j].priority) ?
          assigned[i].task_id : assigned[j].task_id;
        int defer_idx = (assigned[i].task_id == defer_tid) ?
          static_cast<int>(i) : static_cast<int>(j);

        PersistLogger::log_warn("sched.bottleneck_defer",
          assigned[defer_idx].assigned_robot_id, defer_tid,
          "conflicting opposite-direction paths, deferring",
          __FILE__, __LINE__, __func__);

        scheduler_->mark_task_pending(defer_tid);
        auto defer_ni = get_or_create_nav(assigned[defer_idx].assigned_robot_id);
        if (defer_ni) {
          defer_ni->retry_after = now + rclcpp::Duration::from_seconds(5.0);
        }
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(defer_tid);
        if (!ti.task_id.empty()) task_pub_->publish(ti);
        assigned[defer_idx].task_id.clear();
      }
    }
  }

  // ── 启动导航 ──
  for (auto & t : assigned) {
    if (t.task_id.empty()) continue;

    std::string active_target_task = target_has_active_task(t.waypoint_id, t.task_id);
    if (!active_target_task.empty()) {
      defer_assignment(t, "sched.target_active_defer",
        "target " + t.waypoint_id + " already has active task " + active_target_task,
        3.0);
      continue;
    }

    if (has_active_waiters(t.assigned_robot_id)) {
      defer_assignment(t, "sched.blocker_priority_hold",
        "robot is blocking active waiters, delaying assigned task",
        3.0);
      continue;
    }

    auto ni = get_or_create_nav(t.assigned_robot_id);
    if (ni && (ni->has_active_goal || !ni->route.empty() ||
               !ni->current_task_id.empty() || ni->chassis_task_sent)) {
      scheduler_->mark_task_waiting(t.task_id);
      ni->retry_after = now + rclcpp::Duration::from_seconds(3.0);
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
      if (!ti.task_id.empty()) task_pub_->publish(ti);
      continue;
    }

    std::string target_blocker = occupancy_->waypoint_blocker(t.assigned_robot_id, t.waypoint_id);
    std::string target_holder = occupancy_->get_zone_holder(t.waypoint_id);
    if (!target_blocker.empty() && target_blocker != t.assigned_robot_id) {
      waiting_for_[target_blocker].insert(t.assigned_robot_id);
      bool exit_started = false;
      if (target_holder == target_blocker && is_robot_idle(target_holder)) {
        std::set<std::string> exclude;
        auto st = robots_.find(t.assigned_robot_id);
        if (st != robots_.end()) {
          std::string swp = st->second.current_waypoint;
          if (swp.empty()) swp = traffic_->find_nearest_waypoint(st->second.current_pose);
          if (!swp.empty()) {
            auto path = traffic_->find_path(swp, t.waypoint_id);
            for (const auto & wp : path) exclude.insert(wp);
          }
        }
        for (const auto & active_target : active_task_targets(t.task_id)) {
          exclude.insert(active_target);
        }
        std::string exit_wp = find_safe_free_waypoint(t.waypoint_id, exclude, target_holder);
        if (!exit_wp.empty() && occupancy_->reserve_next(target_holder, t.waypoint_id, exit_wp)) {
          auto holder_ni = get_or_create_nav(target_holder);
          if (holder_ni) {
            PersistLogger::log_info("sched.target_exit_reserved", target_holder, t.task_id,
              "moving target holder from " + t.waypoint_id + " to " + exit_wp +
              " for requester " + t.assigned_robot_id,
              __FILE__, __LINE__, __func__);
            holder_ni->route = {exit_wp};
            holder_ni->route_index = 0;
            holder_ni->current_task_id = "avoidance_" + target_holder;
            holder_ni->retry_count = 0;
            holder_ni->retry_after = rclcpp::Time{};
            navigate_to_waypoint(target_holder, exit_wp, holder_ni->current_task_id, true);
            exit_started = true;
          }
        }
        if (!exit_started && !chain_plan_.active) {
          std::vector<RetreatChainStep> clear_steps;
          std::string chain_exit_wp;
          for (const auto & nb : traffic_->get_waypoint_connections(t.waypoint_id)) {
            if (exclude.count(nb)) continue;
            std::string blocker = occupancy_->can_enter(target_holder, t.waypoint_id, nb);
            if (blocker.empty() || blocker == target_holder) {
              chain_exit_wp = nb;
              break;
            }
            auto nb_holder = occupancy_->get_zone_holder(nb);
            if (nb_holder.empty() || !is_robot_stationary(nb_holder)) continue;
            std::set<std::string> push_excluded = exclude;
            push_excluded.insert(t.waypoint_id);
            std::set<std::string> visited;
            std::vector<RetreatChainStep> push_steps;
            if (try_push_occupant(nb, push_excluded, visited, 0, push_steps)) {
              clear_steps = std::move(push_steps);
              chain_exit_wp = nb;
              break;
            }
          }
          if (!chain_exit_wp.empty()) {
            chain_plan_.steps.clear();
            chain_plan_.saved_task_ids.clear();
            chain_plan_.saved_targets.clear();
            for (auto & s : clear_steps) chain_plan_.steps.push_back(std::move(s));
            chain_plan_.steps.push_back({target_holder, chain_exit_wp});
            for (const auto & s : chain_plan_.steps) {
              auto ri = get_or_create_nav(s.robot_id);
              if (!ri || ri->current_task_id.empty()) continue;
              chain_plan_.saved_task_ids[s.robot_id] = ri->current_task_id;
              if (!ri->route.empty()) chain_plan_.saved_targets[s.robot_id] = ri->route.back();
              ri->has_active_goal = false;
              ri->route.clear();
              ri->route_index = 0;
              ri->retry_count = 0;
            }
            chain_plan_.original_requester = t.assigned_robot_id;
            chain_plan_.original_target = t.waypoint_id;
            chain_plan_.original_task_id = t.task_id;
            chain_plan_.active = true;
            chain_plan_.started_at = now;
            chain_plan_.current_step = 0;
            chain_plan_.step_retry_count = 0;
            PersistLogger::log_info("sched.target_exit_chain_started", target_holder, t.task_id,
              "moving target holder from " + t.waypoint_id + " to " + chain_exit_wp +
              " with " + std::to_string(chain_plan_.steps.size()) + " internal steps",
              __FILE__, __LINE__, __func__);
            execute_chain_step();
            exit_started = true;
          } else {
            PersistLogger::log_warn("sched.target_exit_unavailable", target_holder, t.task_id,
              "no safe exit from occupied target " + t.waypoint_id +
              " for requester " + t.assigned_robot_id,
              __FILE__, __LINE__, __func__);
          }
        }
      } else if (!target_holder.empty()) {
        PersistLogger::log_info("sched.target_holder_busy", target_holder, t.task_id,
          "target holder is not idle for " + t.waypoint_id,
          __FILE__, __LINE__, __func__);
      } else {
        PersistLogger::log_info("sched.target_reserved_defer", target_blocker, t.task_id,
          "target " + t.waypoint_id + " reserved by " + target_blocker,
          __FILE__, __LINE__, __func__);
      }
      PersistLogger::log_warn("sched.target_occupied_defer", t.assigned_robot_id, t.task_id,
        "target " + t.waypoint_id + " blocked by " + target_blocker,
        __FILE__, __LINE__, __func__);
      scheduler_->mark_task_waiting(t.task_id);
      task_waits_[t.task_id] = {TaskWaitState::WAIT_TARGET_CLEAR, t.assigned_robot_id,
        t.task_id, target_blocker, std::string{}, std::string{}, t.waypoint_id, 0};
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
      if (!ti.task_id.empty()) task_pub_->publish(ti);
      t.task_id.clear();
      continue;
    }

    PersistLogger::log_info("sched.assign", t.assigned_robot_id, t.task_id,
      "assigned to wp=" + t.waypoint_id, __FILE__, __LINE__, __func__);
    start_navigation(t.assigned_robot_id, t.waypoint_id, t.task_id);
  }
}

// ============================================================================
// 死锁检测
// ============================================================================

void FleetManagerNode::deadlock_check()
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  const auto now = this->now();

  // 构建阻塞图: 包含活跃导航和航点阻塞的底盘
  std::map<std::string, std::string> block_graph;
  for (auto & [rid, ni] : navs_) {
    if (!ni || ni->route.empty() || ni->route_index >= ni->route.size()) continue;
    if (ni->current_task_id.empty()) continue;

    std::string next_wp = ni->route[ni->route_index];
    std::string blocker = occupancy_->waypoint_blocker(rid, next_wp);
    if (!blocker.empty() && blocker != rid) {
      block_graph[rid] = blocker;
    }
  }

  // DFS 环检测
  std::vector<std::string> cycle;
  for (const auto & [rid, _] : block_graph) {
    std::map<std::string, size_t> seen;
    std::vector<std::string> chain;
    std::string cur = rid;
    while (!cur.empty()) {
      if (seen.count(cur)) {
        cycle.assign(chain.begin() + static_cast<long>(seen[cur]), chain.end());
        break;
      }
      seen[cur] = chain.size();
      chain.push_back(cur);
      auto nxt = block_graph.find(cur);
      if (nxt == block_graph.end()) break;
      cur = nxt->second;
    }
    if (cycle.size() >= 2) break;
  }

  if (cycle.size() < 2) {
    prev_cycle_key_.clear();
    cycle_first_seen_ = rclcpp::Time{};
    return;
  }

  // 生成排序后的环特征 key(旋转无关)
  std::string cycle_key;
  {
    std::vector<std::string> sorted = cycle;
    std::sort(sorted.begin(), sorted.end());
    for (const auto & r : sorted) cycle_key += r + "|";
  }

  if (cycle_key != prev_cycle_key_) {
    prev_cycle_key_ = cycle_key;
    cycle_first_seen_ = now;
    PersistLogger::log_info("deadlock.detected", "", "",
      "cycle=" + cycle_key, __FILE__, __LINE__, __func__);
    return;
  }

  double duration = (now - cycle_first_seen_).seconds();
  if (duration < deadlock_timeout_) return;

  // 打破死锁
  deadlock_break_count_++;
  PersistLogger::log_warn("deadlock.break", "", "",
    "deadlock persisted for " + std::to_string(duration) + "s, breaking",
    __FILE__, __LINE__, __func__);

  // 选最低优先级任务的底盘作为 victim
  std::string victim;
  int victim_pri = std::numeric_limits<int>::max();
  for (const auto & rid : cycle) {
    auto ni = navs_.find(rid);
    if (ni == navs_.end() || !ni->second) continue;
    auto ti = scheduler_->get_task_info(ni->second->current_task_id);
    int pri = ti.task_id.empty() ? 0 : ti.priority;
    if (pri < victim_pri || (pri == victim_pri && victim.empty())) {
      victim_pri = pri;
      victim = rid;
    }
  }

  if (victim.empty()) return;

  auto ni = navs_.find(victim);
  if (ni == navs_.end() || !ni->second) return;

  std::string tid = ni->second->current_task_id;
  cancel_goals(ni->second);
  occupancy_->release_reservations(victim);
  ni->second->current_task_id.clear();
  ni->second->route.clear();
  ni->second->route_index = 0;
  ni->second->retry_count = 0;
  stop_robot(victim, 5);

  // 物理移走 victim: 释放 zone_locks 并导航到最近空闲航点
  auto st = robots_.find(victim);
  if (st != robots_.end()) {
    std::string cur_wp = st->second.current_waypoint;
    if (cur_wp.empty())
      cur_wp = traffic_->find_nearest_waypoint(st->second.current_pose);
    if (!cur_wp.empty()) {
      std::string relocate_wp = find_safe_free_waypoint(cur_wp, {}, victim);
      if (!relocate_wp.empty() && relocate_wp != cur_wp) {
        PersistLogger::log_info("deadlock.relocate_victim", victim, tid,
          "relocating victim from " + cur_wp + " to " + relocate_wp,
          __FILE__, __LINE__, __func__);
        ni->second->route = {relocate_wp};
        ni->second->route_index = 0;
        ni->second->current_task_id = "relocate_" + victim;
        ni->second->retry_count = 0;
        ni->second->retry_after = rclcpp::Time{};
        navigate_to_waypoint(victim, relocate_wp, ni->second->current_task_id, true);
      }
    }
  }

  if (!tid.empty()) {
    scheduler_->mark_task_pending(tid);
    fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
    if (!ti.task_id.empty()) task_pub_->publish(ti);
  }

  prev_cycle_key_.clear();
  cycle_first_seen_ = rclcpp::Time{};
}

// ============================================================================
// 运营指标
// ============================================================================

void FleetManagerNode::publish_metrics()
{
  std_msgs::msg::String msg;

  size_t online = 0, offline = 0;
  for (const auto & [_, st] : robots_) {
    if (st.connection_status == "online") online++; else offline++;
  }

  size_t pending = 0, active = 0, completed = 0, failed = 0;
  for (const auto & t : scheduler_->get_all_tasks()) {
    if (t.status == "pending" || t.status == "waiting_fleet" || t.status == "assigned") pending++;
    else if (t.status == "in_progress" || t.status == "executing") active++;
    else if (t.status == "completed") completed++;
    else if (t.status == "failed") failed++;
  }

  std::ostringstream oss;
  oss << "robots_online=" << online
      << " robots_offline=" << offline
      << " tasks_pending=" << pending
      << " tasks_active=" << active
      << " tasks_completed=" << completed
      << " tasks_failed=" << failed
      << " deadlock_breaks=" << deadlock_break_count_;
  msg.data = oss.str();
  metrics_pub_->publish(msg);
}

// ============================================================================
// 图感知安全搜索: 找最近空闲航点，且路径不穿过占用或排除的航点
// ============================================================================

std::string FleetManagerNode::find_safe_free_waypoint(
  const std::string & from_wp,
  const std::set<std::string> & exclude,
  const std::string & self_robot) const
{
  std::string best;
  double best_dist = std::numeric_limits<double>::max();
  auto from_pose = traffic_->get_waypoint_pose(from_wp);
  auto all_wps = traffic_->get_all_waypoint_poses();

  for (const auto & [wp_id, wp_pose] : all_wps) {
    if (wp_id == from_wp || exclude.count(wp_id)) continue;
    if (!occupancy_->is_zone_free_for("", wp_id)) continue;

    auto path = traffic_->find_path(from_wp, wp_id);
    if (path.empty()) continue;

    bool blocked = false;
    for (const auto & w : path) {
      if (w == from_wp) continue;
      if (exclude.count(w)) { blocked = true; break; }
      auto holder = occupancy_->get_zone_holder(w);
      if (!holder.empty() && holder != self_robot) { blocked = true; break; }
    }
    if (blocked) continue;

    double d = std::hypot(wp_pose.position.x - from_pose.position.x,
                           wp_pose.position.y - from_pose.position.y);
    if (d < best_dist) { best_dist = d; best = wp_id; }
  }
  return best;
}

// ============================================================================
// 事件驱动唤醒: blocker 完成后主动唤醒等待它的请求者
// ============================================================================

void FleetManagerNode::wake_waiters(const std::string & blocker_id)
{
  auto it = waiting_for_.find(blocker_id);
  if (it == waiting_for_.end() || it->second.empty()) return;

  PersistLogger::log_info("sched.wake_waiters", blocker_id, "",
    "waking " + std::to_string(it->second.size()) + " waiters",
    __FILE__, __LINE__, __func__);

  auto waiters = std::move(it->second);
  waiting_for_.erase(it);

  std::vector<std::string> ready_tasks;
  for (const auto & [task_id, wait] : task_waits_) {
    if (wait.blocker_id != blocker_id) continue;
    bool ready = false;
    if (wait.state == TaskWaitState::WAIT_TARGET_CLEAR) {
      ready = occupancy_->waypoint_blocker(wait.robot_id, wait.target_wp).empty();
    } else if (wait.from_wp.empty() || wait.to_wp.empty()) {
      ready = true;
    } else {
      auto blocker = occupancy_->can_enter(wait.robot_id, wait.from_wp, wait.to_wp);
      ready = blocker.empty() || blocker != blocker_id;
    }
    if (ready) ready_tasks.push_back(task_id);
  }
  for (const auto & task_id : ready_tasks) {
    auto wait = task_waits_[task_id];
    task_waits_.erase(task_id);
    auto ni = get_or_create_nav(wait.robot_id);
    if (ni) {
      ni->retry_count = 0;
      ni->retry_after = rclcpp::Time{};
    }
    PersistLogger::log_info("sched.wait_condition_ready", wait.robot_id, task_id,
      "blocker " + blocker_id + " changed, task can be redispatched",
      __FILE__, __LINE__, __func__);
  }

  for (const auto & waiter_id : waiters) {
    auto ni = get_or_create_nav(waiter_id);
    if (!ni || ni->current_task_id.empty() || ni->route.empty()) continue;

    size_t target = std::min(ni->route_index, ni->route.size() - 1);
    std::string to = ni->route[target];
    std::string from;
    if (target > 0) {
      from = ni->route[target - 1];
    } else {
      auto st = robots_.find(waiter_id);
      if (st != robots_.end()) {
        from = st->second.current_waypoint;
        if (from.empty()) from = traffic_->find_nearest_waypoint(st->second.current_pose);
      }
    }
    if (!from.empty() && from != to) {
      auto blocker = occupancy_->can_enter(waiter_id, from, to);
      if (!blocker.empty()) {
        waiting_for_[blocker].insert(waiter_id);
        PersistLogger::log_info("sched.waiter_still_blocked", waiter_id, ni->current_task_id,
          "next hop " + from + "->" + to + " still blocked by " + blocker,
          __FILE__, __LINE__, __func__);
        continue;
      }
    }

    ni->retry_count = 0;
    ni->retry_after = rclcpp::Time{};
    PersistLogger::log_info("sched.waiter_retry", waiter_id, ni->current_task_id,
      "blocker " + blocker_id + " finished, retrying navigation",
      __FILE__, __LINE__, __func__);
    navigate_to_next_waypoint(waiter_id);
  }
}

}  // namespace fleet_manager
