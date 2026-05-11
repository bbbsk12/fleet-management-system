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
  this->declare_parameter("deadlock_timeout_sec", 5.0);
  this->declare_parameter("max_task_retry_cycles", 8);

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
  // 链 retreat 只阻塞链的参与者(steps + saved_task_ids),其它机器人
  // 继续正常调度,避免整队被一条 chain 卡死。
  std::set<std::string> chain_participants;
  if (chain_plan_.active) {
    for (const auto & s : chain_plan_.steps) chain_participants.insert(s.robot_id);
    for (const auto & [rid, _] : chain_plan_.saved_task_ids) {
      (void)_;
      chain_participants.insert(rid);
    }
    if (!chain_participants.empty()) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "chain retreat active, skipping %zu participant(s)", chain_participants.size());
    }
  }
  auto is_chain_participant = [&](const std::string & robot_id) {
    return !robot_id.empty() && chain_participants.count(robot_id) > 0;
  };

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
  FleetStateSnapshot snapshot_cache;
  bool snapshot_dirty = true;
  auto current_snapshot = [&]() -> const FleetStateSnapshot & {
    if (snapshot_dirty) {
      snapshot_cache = build_fleet_state_snapshot();
      snapshot_dirty = false;
    }
    return snapshot_cache;
  };
  auto mark_snapshot_dirty = [&]() {
    snapshot_dirty = true;
  };
  auto has_active_waiters = [&](const std::string & blocker_id) {
    const auto & snapshot = current_snapshot();
    for (const auto & edge : snapshot.wait_edges) {
      if (edge.blocker_robot_id == blocker_id) return true;
    }
    return false;
  };
  auto task_releases_waiters = [&](const std::string & blocker_id,
      const std::string & task_target_wp) {
    std::string current_wp;
    auto st = robots_.find(blocker_id);
    if (st != robots_.end()) {
      current_wp = st->second.current_waypoint;
      if (current_wp.empty()) current_wp = traffic_->find_nearest_waypoint(st->second.current_pose);
    }
    auto route = plan_route_for_task(blocker_id, task_target_wp);
    const bool leaves_current_wp = !current_wp.empty() &&
      task_target_wp != current_wp && route.size() >= 2;
    bool found = false;
    const auto & snapshot = current_snapshot();
    for (const auto & edge : snapshot.wait_edges) {
      if (edge.blocker_robot_id != blocker_id) continue;
      found = true;
      if (edge.wait_state == TaskWaitState::WAIT_TARGET_CLEAR) {
        if (edge.target_wp.empty() || edge.target_wp == task_target_wp) return false;
        continue;
      }
      if (!leaves_current_wp) return false;
      if (!edge.to_wp.empty() && edge.to_wp == current_wp) continue;
      if (!edge.resource.empty()) {
        if (edge.resource == current_wp) continue;
        const auto arrow = edge.resource.find("->");
        if (arrow != std::string::npos) {
          const auto from = edge.resource.substr(0, arrow);
          const auto to = edge.resource.substr(arrow + 2);
          if (from == current_wp || to == current_wp) continue;
        }
      }
      return false;
    }
    return found;
  };
  auto target_has_active_task = [&](const std::string & target_wp,
      const std::string & current_task_id) {
    for (const auto & existing : scheduler_->get_all_tasks()) {
      if (existing.task_id.empty() || existing.task_id == current_task_id) continue;
      if (existing.waypoint_id != target_wp) continue;
      // 终结/排队中/被 defer 的任务都不算"占据" target:
      // pending  = 在队列里没分配;
      // waiting_fleet = 自己被 defer/wait,没有真正 progress
      //   (若它的 wait 反而依赖目标 wp,物理占用会由 occupancy 检查兜底)。
      // 排除 waiting_fleet 防止两个同 target 任务互相 target_active_defer 死锁。
      if (existing.status == "completed" || existing.status == "failed" ||
          existing.status == "cancelled" || existing.status == "pending" ||
          existing.status == "waiting_fleet") continue;
      return existing.task_id;
    }
    return std::string{};
  };
  // 判定 "占着 target 的 active task 是否真的卡住":
  // 如果它的机器人 has_active_goal/chassis_task_sent → 在正常推进,等待者应耐心等;
  // 如果它的机器人 idle 且不是 chain participant → 才视为 stalled,允许 escape。
  // 这个判定避免把"排队等正常 in_progress 任务"误杀为死锁。
  auto is_active_task_stalled = [&](const std::string & active_task_id) -> bool {
    if (active_task_id.empty()) return false;
    fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(active_task_id);
    if (ti.task_id.empty() || ti.assigned_robot_id.empty()) return false;
    auto it = navs_.find(ti.assigned_robot_id);
    if (it == navs_.end() || !it->second) return true;  // 没 nav 实例视为卡
    const auto & ani = it->second;
    if (ani->has_active_goal) return false;          // nav 在跑
    if (ani->chassis_task_sent) return false;        // chassis 在做
    if (chain_plan_.active &&
        chain_plan_.saved_task_ids.count(ti.assigned_robot_id)) {
      return false;  // 它是 chain participant,系统在解决
    }
    // chain step 也算
    if (chain_plan_.active) {
      for (const auto & s : chain_plan_.steps) {
        if (s.robot_id == ti.assigned_robot_id) return false;
      }
    }
    return true;
  };
  // Robot 已经在 target 附近(no-op task) → 立即完成,避免 hotspot_merge 把
  // holder 自身的同点任务永久 defer,从而吃掉它后面那批会真正离开的任务。
  auto robot_already_at_target = [&](const std::string & robot_id,
      const std::string & target_wp) {
    if (robot_id.empty() || target_wp.empty()) return false;
    auto rs = robots_.find(robot_id);
    if (rs == robots_.end()) return false;
    auto target_pose = traffic_->get_waypoint_pose(target_wp);
    const double dx = rs->second.current_pose.position.x - target_pose.position.x;
    const double dy = rs->second.current_pose.position.y - target_pose.position.y;
    return std::hypot(dx, dy) <= waypoint_radius_;
  };
  auto complete_no_op_task = [&](const std::string & robot_id,
      const std::string & task_id, const std::string & target_wp) {
    if (task_id.empty() || robot_id.empty()) return;
    PersistLogger::log_info("sched.task_already_at_target", robot_id, task_id,
      "robot already at " + target_wp + ", completing immediately",
      __FILE__, __LINE__, __func__);
    clear_task_wait_condition(task_id);
    scheduler_->complete_task(task_id);
    fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
    if (!ti.task_id.empty()) task_pub_->publish(ti);
    auto noop_ni = get_or_create_nav(robot_id);
    if (noop_ni && noop_ni->current_task_id == task_id) {
      noop_ni->current_task_id.clear();
      noop_ni->route.clear();
      noop_ni->route_index = 0;
    }
    wake_waiters(robot_id);
    mark_snapshot_dirty();
  };
  auto active_task_targets = [&](const std::string & current_task_id) {
    std::set<std::string> targets;
    for (const auto & existing : scheduler_->get_all_tasks()) {
      if (existing.task_id.empty() || existing.task_id == current_task_id) continue;
      if (existing.waypoint_id.empty()) continue;
      if (existing.status != "in_progress" &&
          existing.status != "executing" &&
          existing.status != "assigned") {
        continue;
      }
      targets.insert(existing.waypoint_id);
    }
    return targets;
  };
  auto wait_condition_ready = [&](const fleet_msgs::msg::TaskInfo & t) {
    auto it = task_waits_.find(t.task_id);
    if (it == task_waits_.end()) return true;
    auto & w = it->second;
    std::string resource;
    auto blocker = resolve_wait_blocker(w, resource);
    bool ready = blocker.empty() ||
      ((w.state == TaskWaitState::WAIT_BLOCKER_RELEASE ||
        w.state == TaskWaitState::SELF_RELOCATING) &&
       blocker != w.blocker_id);
    if (!ready && blocker != w.blocker_id) {
      w.blocker_id = blocker;
      waiting_for_[blocker].insert(w.robot_id);
      mark_snapshot_dirty();
    }
    if (ready) {
      PersistLogger::log_info("sched.wait_condition_ready", w.robot_id, w.task_id,
        "state ready for task dispatch",
        __FILE__, __LINE__, __func__);
      clear_task_wait_condition(t.task_id);
      mark_snapshot_dirty();
    }
    return ready;
  };
  auto progress_wait_conditions = [&]() {
    for (auto & [task_id, wait] : task_waits_) {
      std::string wait_resource;
      auto current_blocker = resolve_wait_blocker(wait, wait_resource);
      if (current_blocker.empty()) continue;
      if (current_blocker != wait.blocker_id) {
        if (wait.state == TaskWaitState::WAIT_BLOCKER_RELEASE ||
            wait.state == TaskWaitState::SELF_RELOCATING) {
          continue;
        }
        wait.blocker_id = current_blocker;
        waiting_for_[current_blocker].insert(wait.robot_id);
        mark_snapshot_dirty();
        continue;
      }

      if (wait.state == TaskWaitState::WAIT_TARGET_CLEAR) {
        if (wait.target_wp.empty()) continue;
        auto target_holder = occupancy_->get_zone_holder(wait.target_wp);
        if (target_holder.empty() || target_holder == wait.robot_id) continue;
        if (target_holder != wait.blocker_id) continue;

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
          transit_exclude.insert(active_target);
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
                  if (holder_path[j] != holder_candidate &&
                      transit_exclude.count(holder_path[j])) {
                    holder_blocked = true;
                    break;
                  }
                  auto hop_blocker = occupancy_->can_enter(target_holder, holder_path[j - 1], holder_path[j]);
                  if (!hop_blocker.empty() && hop_blocker != target_holder &&
                      hop_blocker != wait.robot_id) {
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
              waiter_ni->route_alignment_done = false;
              waiter_ni->current_task_id = "avoidance_" + wait.robot_id;
              waiter_ni->retry_count = 0;
              waiter_ni->retry_after = rclcpp::Time{};
              wait.retreat_count++;
              mark_snapshot_dirty();
              PersistLogger::log_info("sched.target_clear_requester_relocate", wait.robot_id, task_id,
                "moving requester from " + waiter_wp + " to " + relocate_path.back() +
                " to open exit for target holder " + target_holder,
                __FILE__, __LINE__, __func__);
              navigate_to_next_waypoint(wait.robot_id);
              continue;
            }
          }

          // #1 Fallback: 单跳 exit + requester relocate 都失败 → 尝试 chain retreat
          //    把 target_holder 通过 chain push 推出 target_wp,而不是被动等死锁兜底。
          if (!chain_plan_.active &&
              try_start_target_exit_chain(target_holder, wait.target_wp,
                                          wait.robot_id, task_id, final_exclude))
          {
            mark_snapshot_dirty();
            continue;
          }

          if (log_throttle_ok("target_clear_exit_unavailable:" + target_holder +
              ":" + wait.target_wp + ":" + wait.robot_id, 5.0)) {
            PersistLogger::log_warn("sched.target_clear_exit_unavailable", target_holder, task_id,
              "no exit for target holder at " + wait.target_wp +
              " for requester " + wait.robot_id,
              __FILE__, __LINE__, __func__);
          }
          continue;
        }

        auto holder_ni = get_or_create_nav(target_holder);
        if (!holder_ni) continue;
        holder_ni->route = exit_path;
        holder_ni->route_index = 0;
        holder_ni->route_alignment_done = false;
        holder_ni->current_task_id = "avoidance_" + target_holder;
        holder_ni->retry_count = 0;
        holder_ni->retry_after = rclcpp::Time{};
        waiting_for_[target_holder].insert(wait.robot_id);
        mark_snapshot_dirty();
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
      blocker_ni->route_alignment_done = false;
      blocker_ni->current_task_id = "avoidance_" + wait.blocker_id;
      blocker_ni->retry_count = 0;
      blocker_ni->retry_after = rclcpp::Time{};
      wait.state = TaskWaitState::WAIT_BLOCKER_RELEASE;
      mark_snapshot_dirty();
      PersistLogger::log_info("sched.blocker_exit_reserved", wait.blocker_id, task_id,
        "moving blocker from " + blocker_wp + " to " + exit_path.back() +
        " for waiter " + wait.robot_id,
        __FILE__, __LINE__, __func__);
      navigate_to_next_waypoint(wait.blocker_id);
    }
  };
  progress_wait_conditions();
  mark_snapshot_dirty();

  auto register_route_wait = [&](const std::string & rid,
      const std::string & task_id,
      const std::string & target_wp,
      const std::vector<std::string> & path,
      const std::string & blocker,
      const std::string & conflict_task,
      const std::string & conflict_resource,
      const std::string & tag) {
    if (task_id.empty() || blocker.empty()) return;
    scheduler_->mark_task_waiting(task_id);
    clear_task_wait_condition(task_id);
    task_waits_[task_id] = {TaskWaitState::WAIT_ROUTE_CLEAR, rid, task_id,
      blocker, path.empty() ? std::string{} : path.front(),
      path.size() > 1 ? path[1] : std::string{}, target_wp, 0};
    waiting_for_[blocker].insert(rid);
    mark_snapshot_dirty();
    PersistLogger::log_info(tag, rid, task_id,
      "route " + join_waypoints(path) + " waits for blocking task " + conflict_task +
      " on " + blocker + " resource=" + conflict_resource,
      __FILE__, __LINE__, __func__);
    fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(task_id);
    if (!ti.task_id.empty()) task_pub_->publish(ti);
  };

  // ── waiting_fleet 重调度: 退避期满的 deferred 任务恢复执行 ──
  for (const auto & t : scheduler_->get_all_tasks()) {
    if (t.status != "waiting_fleet") continue;
    if (t.assigned_robot_id.empty()) continue;
    if (is_chain_participant(t.assigned_robot_id)) continue;

    const std::string & rid = t.assigned_robot_id;
    auto ni = get_or_create_nav(rid);
    if (!ni) continue;

    // 跳过已被其他任务占用的底盘
    if (!ni->current_task_id.empty() && ni->current_task_id != t.task_id) continue;
    if (ni->has_active_goal || !ni->route.empty() || ni->chassis_task_sent) continue;
    if (!wait_condition_ready(t)) continue;
    if (ni->retry_after.nanoseconds() > 0 && now < ni->retry_after) continue;

    if (robot_already_at_target(rid, t.waypoint_id)) {
      complete_no_op_task(rid, t.task_id, t.waypoint_id);
      continue;
    }

    std::string active_target_task = target_has_active_task(t.waypoint_id, t.task_id);
    if (!active_target_task.empty()) {
      // #4 累计 escape:仅当 active task 真的卡住(机器人 idle 且非 chain participant)
      // 时才 fail; 否则 active task 在正常推进,等待者应耐心等。
      const bool stalled = is_active_task_stalled(active_target_task);
      int & dcnt = target_active_defer_count_[t.task_id];
      if (stalled) ++dcnt;
      else dcnt = std::max(dcnt - 1, 0);  // active 在动 → 计数衰减
      if (stalled && dcnt >= 30) {
        PersistLogger::log_error("sched.target_active_defer_exhausted", rid, t.task_id,
          "target " + t.waypoint_id + " blocked by stalled " + active_target_task +
          " for " + std::to_string(dcnt) + " ticks, failing task to release robot",
          __FILE__, __LINE__, __func__);
        scheduler_->fail_task(t.task_id,
          "target blocked by stalled " + active_target_task);
        target_active_defer_count_.erase(t.task_id);
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
        if (!ti.task_id.empty()) task_pub_->publish(ti);
        mark_snapshot_dirty();
        continue;
      }
      if (log_throttle_ok("target_active_defer:" + t.task_id + ":" + t.waypoint_id, 5.0)) {
        PersistLogger::log_info("sched.target_active_defer", rid, t.task_id,
          "target " + t.waypoint_id + " already has active task " + active_target_task +
          " (defer#" + std::to_string(dcnt) + (stalled ? " stalled" : " healthy") + ")",
          __FILE__, __LINE__, __func__);
      }
      continue;
    }
    target_active_defer_count_.erase(t.task_id);  // 一旦能进 → 清零

    std::string target_blocker = occupancy_->waypoint_blocker(rid, t.waypoint_id);
    if (!target_blocker.empty()) {
      clear_task_wait_condition(t.task_id);
      task_waits_[t.task_id] = {TaskWaitState::WAIT_TARGET_CLEAR, rid,
        t.task_id, target_blocker, std::string{}, std::string{}, t.waypoint_id, 0};
      waiting_for_[target_blocker].insert(rid);
      mark_snapshot_dirty();
      PersistLogger::log_info("sched.target_blocked_defer", rid, t.task_id,
        "target " + t.waypoint_id + " blocked by " + target_blocker,
        __FILE__, __LINE__, __func__);
      continue;
    }

    auto route_path = plan_route_for_task(rid, t.waypoint_id);
    std::string route_conflict_task;
    std::string route_conflict_resource;
    auto route_blocker = find_active_route_conflict(rid, t.task_id, route_path,
      route_conflict_task, route_conflict_resource, true);
    if (!route_blocker.empty()) {
      register_route_wait(rid, t.task_id, t.waypoint_id, route_path,
        route_blocker, route_conflict_task, route_conflict_resource,
        "sched.route_active_defer");
      continue;
    }

    auto st = robots_.find(rid);
    if (st == robots_.end() || st->second.connection_status != "online") continue;
    const bool active_waiters = has_active_waiters(rid);
    const std::string phk = rid + ":" + t.task_id;
    if (active_waiters && !task_releases_waiters(rid, t.waypoint_id)) {
      // 累计 hold N 次后 bypass: 协调式假设别人会让路,但当 chain retreat 同时
      // 失败时两边都 hold → 永久死锁。bypass 让 nav 层 hop_blocked / chain 重新评估。
      int & hcnt = priority_hold_count_[phk];
      if (++hcnt >= 10) {
        PersistLogger::log_warn("sched.blocker_priority_hold_bypass", rid, t.task_id,
          "hold persisted " + std::to_string(hcnt) +
          " ticks, bypassing to let nav layer handle",
          __FILE__, __LINE__, __func__);
        priority_hold_count_.erase(phk);
        // fall through to dispatch
      } else {
        if (log_throttle_ok("blocker_priority_hold:" + t.task_id, 5.0)) {
          PersistLogger::log_info("sched.blocker_priority_hold", rid, t.task_id,
            "robot is blocking active waiters, delaying deferred task (hold#" +
            std::to_string(hcnt) + ")",
            __FILE__, __LINE__, __func__);
        }
        continue;
      }
    } else {
      priority_hold_count_.erase(phk);
    }
    if (active_waiters) {
      PersistLogger::log_info("sched.blocker_release_task", rid, t.task_id,
        "allowing task to move blocker away from waited target",
        __FILE__, __LINE__, __func__);
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
      mark_snapshot_dirty();
      continue;
    }

    PersistLogger::log_info("sched.redispatch_waiting", rid, t.task_id,
      "retrying deferred task to wp=" + t.waypoint_id, __FILE__, __LINE__, __func__);
    start_navigation(rid, t.waypoint_id, t.task_id);
    mark_snapshot_dirty();
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
    if (!tag.empty()) {
      PersistLogger::log_warn(tag, t.assigned_robot_id, t.task_id,
        reason, __FILE__, __LINE__, __func__);
    }
    scheduler_->mark_task_pending(t.task_id);
    auto defer_ni = get_or_create_nav(t.assigned_robot_id);
    if (defer_ni &&
        !defer_ni->has_active_goal &&
        defer_ni->current_task_id.empty() &&
        defer_ni->route.empty() &&
        !defer_ni->chassis_task_sent) {
      defer_ni->retry_after = now + rclcpp::Duration::from_seconds(retry_sec);
    }
    fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
    if (!ti.task_id.empty()) task_pub_->publish(ti);
    t.task_id.clear();
  };
  std::map<std::string, std::string> claimed_targets;
  std::map<std::string, std::string> claimed_hubs;
  std::map<std::string, std::pair<std::string, std::string>> claimed_route_wps;
  std::map<std::string, std::pair<std::string, std::string>> claimed_route_edges;
  auto directed_edge_key = [](const std::string & from, const std::string & to) {
    return from + "->" + to;
  };
  for (auto & t : assigned) {
    if (t.task_id.empty()) continue;
    if (is_chain_participant(t.assigned_robot_id)) {
      // 参与链的机器人不参与本批分配,把任务推回 pending 让链跑完再说
      const bool emit = log_throttle_ok("chain_participant_defer:" + t.task_id, 5.0);
      defer_assignment(t, emit ? "sched.chain_participant_defer" : "",
        "robot is participating in retreat chain", 1.0);
      continue;
    }

    if (robot_already_at_target(t.assigned_robot_id, t.waypoint_id)) {
      complete_no_op_task(t.assigned_robot_id, t.task_id, t.waypoint_id);
      t.task_id.clear();
      continue;
    }

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

    std::string route_conflict_resource;
    std::string route_conflict_task;
    std::string route_conflict_robot;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      auto edge_claim = claimed_route_edges.find(directed_edge_key(path[i + 1], path[i]));
      if (edge_claim != claimed_route_edges.end()) {
        route_conflict_resource = directed_edge_key(path[i], path[i + 1]);
        route_conflict_task = edge_claim->second.first;
        route_conflict_robot = edge_claim->second.second;
        break;
      }
    }
    if (route_conflict_robot.empty()) {
      const size_t first_claimed_wp = path.size() > 1 ? 1 : 0;
      for (size_t i = first_claimed_wp; i < path.size(); ++i) {
        auto wp_claim = claimed_route_wps.find(path[i]);
        if (wp_claim != claimed_route_wps.end()) {
          route_conflict_resource = path[i];
          route_conflict_task = wp_claim->second.first;
          route_conflict_robot = wp_claim->second.second;
          break;
        }
      }
    }
    if (!route_conflict_robot.empty()) {
      register_route_wait(t.assigned_robot_id, t.task_id, t.waypoint_id, path,
        route_conflict_robot, route_conflict_task, route_conflict_resource,
        "sched.route_batch_defer");
      t.task_id.clear();
      continue;
    }

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
    const size_t first_claimed_wp = path.size() > 1 ? 1 : 0;
    for (size_t i = first_claimed_wp; i < path.size(); ++i) {
      claimed_route_wps[path[i]] = {t.task_id, t.assigned_robot_id};
    }
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      claimed_route_edges[directed_edge_key(path[i], path[i + 1])] =
        {t.task_id, t.assigned_robot_id};
    }
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
    if (is_chain_participant(t.assigned_robot_id)) {
      defer_assignment(t, "sched.chain_participant_defer",
        "robot is participating in retreat chain", 1.0);
      continue;
    }

    if (robot_already_at_target(t.assigned_robot_id, t.waypoint_id)) {
      complete_no_op_task(t.assigned_robot_id, t.task_id, t.waypoint_id);
      t.task_id.clear();
      continue;
    }

    std::string active_target_task = target_has_active_task(t.waypoint_id, t.task_id);
    if (!active_target_task.empty()) {
      // #4 累计 escape:仅当 active task stalled 才计数+escape
      const bool stalled = is_active_task_stalled(active_target_task);
      int & dcnt = target_active_defer_count_[t.task_id];
      if (stalled) ++dcnt;
      else dcnt = std::max(dcnt - 1, 0);
      if (stalled && dcnt >= 30) {
        PersistLogger::log_error("sched.target_active_defer_exhausted",
          t.assigned_robot_id, t.task_id,
          "target " + t.waypoint_id + " blocked by stalled " + active_target_task +
          " for " + std::to_string(dcnt) + " ticks, failing task to release robot",
          __FILE__, __LINE__, __func__);
        scheduler_->fail_task(t.task_id,
          "target blocked by stalled " + active_target_task);
        target_active_defer_count_.erase(t.task_id);
        fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
        if (!ti.task_id.empty()) task_pub_->publish(ti);
        mark_snapshot_dirty();
        t.task_id.clear();
        continue;
      }
      const bool emit = log_throttle_ok("target_active_defer:" + t.task_id +
        ":" + t.waypoint_id, 5.0);
      defer_assignment(t, emit ? "sched.target_active_defer" : "",
        "target " + t.waypoint_id + " already has active task " + active_target_task +
        " (defer#" + std::to_string(dcnt) + (stalled ? " stalled" : " healthy") + ")",
        3.0);
      continue;
    }
    target_active_defer_count_.erase(t.task_id);  // 进度推进 → 清零

    const bool active_waiters = has_active_waiters(t.assigned_robot_id);
    const std::string phk2 = t.assigned_robot_id + ":" + t.task_id;
    if (active_waiters &&
        !task_releases_waiters(t.assigned_robot_id, t.waypoint_id)) {
      // 累计 hold N 次后 bypass,同 waiting_fleet 路径
      int & hcnt = priority_hold_count_[phk2];
      if (++hcnt >= 10) {
        PersistLogger::log_warn("sched.blocker_priority_hold_bypass",
          t.assigned_robot_id, t.task_id,
          "hold persisted " + std::to_string(hcnt) +
          " ticks, bypassing to let nav layer handle",
          __FILE__, __LINE__, __func__);
        priority_hold_count_.erase(phk2);
        // fall through to dispatch
      } else {
        const bool emit = log_throttle_ok("blocker_priority_hold:" + t.task_id, 5.0);
        defer_assignment(t, emit ? "sched.blocker_priority_hold" : "",
          "robot is blocking active waiters, delaying assigned task (hold#" +
          std::to_string(hcnt) + ")",
          3.0);
        continue;
      }
    } else {
      priority_hold_count_.erase(phk2);
    }
    if (active_waiters) {
      PersistLogger::log_info("sched.blocker_release_task", t.assigned_robot_id, t.task_id,
        "allowing task to move blocker away from waited target",
        __FILE__, __LINE__, __func__);
    }

    auto ni = get_or_create_nav(t.assigned_robot_id);
    if (ni && (ni->has_active_goal || !ni->route.empty() ||
               !ni->current_task_id.empty() || ni->chassis_task_sent)) {
      scheduler_->mark_task_waiting(t.task_id);
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
      if (!ti.task_id.empty()) task_pub_->publish(ti);
      continue;
    }

    std::string target_blocker = occupancy_->waypoint_blocker(t.assigned_robot_id, t.waypoint_id);
    std::string target_holder = occupancy_->get_zone_holder(t.waypoint_id);
    if (!target_blocker.empty() && target_blocker != t.assigned_robot_id) {
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
        auto exit_path = exit_wp.empty() ? std::vector<std::string>{} :
          traffic_->find_path(t.waypoint_id, exit_wp);
        if (exit_path.size() >= 2 &&
            occupancy_->reserve_next(target_holder, exit_path[0], exit_path[1])) {
          auto holder_ni = get_or_create_nav(target_holder);
          if (holder_ni) {
            PersistLogger::log_info("sched.target_exit_reserved", target_holder, t.task_id,
              "moving target holder from " + t.waypoint_id + " to " + exit_wp +
              " for requester " + t.assigned_robot_id,
              __FILE__, __LINE__, __func__);
            holder_ni->route = exit_path;
            holder_ni->route_index = 0;
            holder_ni->route_alignment_done = false;
            holder_ni->current_task_id = "avoidance_" + target_holder;
            holder_ni->retry_count = 0;
            holder_ni->retry_after = rclcpp::Time{};
            navigate_to_next_waypoint(target_holder);
            mark_snapshot_dirty();
            exit_started = true;
          }
        }
        if (!exit_started && !chain_plan_.active) {
          if (try_start_target_exit_chain(target_holder, t.waypoint_id,
                                          t.assigned_robot_id, t.task_id, exclude))
          {
            mark_snapshot_dirty();
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
      clear_task_wait_condition(t.task_id);
      task_waits_[t.task_id] = {TaskWaitState::WAIT_TARGET_CLEAR, t.assigned_robot_id,
        t.task_id, target_blocker, std::string{}, std::string{}, t.waypoint_id, 0};
      waiting_for_[target_blocker].insert(t.assigned_robot_id);
      mark_snapshot_dirty();
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
      if (!ti.task_id.empty()) task_pub_->publish(ti);
      t.task_id.clear();
      continue;
    }

    auto route_path = plan_route_for_task(t.assigned_robot_id, t.waypoint_id);
    std::string route_conflict_task;
    std::string route_conflict_resource;
    auto route_blocker = find_active_route_conflict(t.assigned_robot_id, t.task_id,
      route_path, route_conflict_task, route_conflict_resource, true);
    if (!route_blocker.empty()) {
      register_route_wait(t.assigned_robot_id, t.task_id, t.waypoint_id,
        route_path, route_blocker, route_conflict_task, route_conflict_resource,
        "sched.route_active_defer");
      t.task_id.clear();
      continue;
    }

    PersistLogger::log_info("sched.assign", t.assigned_robot_id, t.task_id,
      "assigned to wp=" + t.waypoint_id, __FILE__, __LINE__, __func__);
    start_navigation(t.assigned_robot_id, t.waypoint_id, t.task_id);
    mark_snapshot_dirty();
  }
}

// ============================================================================
// 死锁检测
// ============================================================================

void FleetManagerNode::clear_task_wait_condition(const std::string & task_id)
{
  auto wait_it = task_waits_.find(task_id);
  if (wait_it == task_waits_.end()) return;
  const auto waiter = wait_it->second.robot_id;
  for (auto it = waiting_for_.begin(); it != waiting_for_.end(); ) {
    it->second.erase(waiter);
    if (it->second.empty()) {
      it = waiting_for_.erase(it);
    } else {
      ++it;
    }
  }
  task_waits_.erase(wait_it);
}

std::string FleetManagerNode::resolve_wait_blocker(
  const TaskWaitCondition & wait, std::string & resource) const
{
  resource.clear();
  if (wait.state == TaskWaitState::WAIT_TARGET_CLEAR) {
    resource = wait.target_wp;
    auto blocker = occupancy_->waypoint_blocker(wait.robot_id, wait.target_wp);
    if (blocker.empty()) {
      blocker = physical_waypoint_blocker(wait.robot_id, wait.target_wp);
    }
    return blocker;
  }
  if (wait.state == TaskWaitState::WAIT_ROUTE_CLEAR) {
    std::string conflict_task;
    auto blocker = find_active_route_conflict(wait.robot_id, wait.task_id,
      plan_route_for_task(wait.robot_id, wait.target_wp), conflict_task, resource, true);
    return blocker;
  }
  if (!wait.from_wp.empty() && !wait.to_wp.empty()) {
    resource = wait.from_wp + "->" + wait.to_wp;
    auto blocker = occupancy_->can_enter(wait.robot_id, wait.from_wp, wait.to_wp);
    if (blocker.empty()) {
      blocker = physical_waypoint_blocker(wait.robot_id, wait.to_wp);
    }
    return blocker;
  }
  return wait.blocker_id;
}

FleetStateSnapshot FleetManagerNode::build_fleet_state_snapshot() const
{
  FleetStateSnapshot snapshot;
  auto add_wait_edge = [&](const WaitEdge & edge) {
    if (edge.waiter_robot_id.empty() || edge.blocker_robot_id.empty() ||
        edge.waiter_robot_id == edge.blocker_robot_id) {
      return;
    }
    snapshot.wait_edges.push_back(edge);
    if (!snapshot.wait_graph.count(edge.waiter_robot_id) || edge.active_navigation) {
      snapshot.wait_graph[edge.waiter_robot_id] = edge.blocker_robot_id;
    }
    auto & robot = snapshot.robots[edge.waiter_robot_id];
    robot.robot_id = edge.waiter_robot_id;
    robot.blocker_id = edge.blocker_robot_id;
    robot.wait_resource = edge.resource;
  };

  for (const auto & [rid, st] : robots_) {
    RobotStateSnapshot robot;
    robot.robot_id = rid;
    robot.online = st.connection_status == "online";
    robot.current_wp = st.current_waypoint;
    if (robot.current_wp.empty() && robot.online) {
      robot.current_wp = traffic_->find_nearest_waypoint(st.current_pose);
    }
    robot.state = robot.online ? RobotMotionState::IDLE : RobotMotionState::OFFLINE;
    auto resource_state = occupancy_->get_robot_resource_state(rid);
    if (resource_state == RobotResourceState::CONFLICT) robot.state = RobotMotionState::CONFLICT;
    else if (resource_state == RobotResourceState::GHOST) robot.state = RobotMotionState::GHOST;
    snapshot.robots[rid] = robot;
  }

  for (auto & [rid, ni] : navs_) {
    auto & robot = snapshot.robots[rid];
    robot.robot_id = rid;
    auto st = robots_.find(rid);
    robot.online = st != robots_.end() && st->second.connection_status == "online";
    if (robot.current_wp.empty() && st != robots_.end()) {
      robot.current_wp = st->second.current_waypoint;
      if (robot.current_wp.empty() && robot.online) {
        robot.current_wp = traffic_->find_nearest_waypoint(st->second.current_pose);
      }
    }
    if (!ni) continue;
    robot.task_id = ni->current_task_id;
    robot.route = ni->route;
    robot.route_index = ni->route_index;
    robot.has_active_goal = ni->has_active_goal;
    robot.chassis_task_sent = ni->chassis_task_sent;
    if (!ni->route.empty() && ni->route_index < ni->route.size()) {
      robot.next_wp = ni->route[ni->route_index];
      robot.target_wp = ni->route.back();
    }
    if (!robot.online) {
      robot.state = RobotMotionState::OFFLINE;
    } else if (ni->chassis_task_sent) {
      robot.state = RobotMotionState::EXECUTING;
    } else if (ni->current_task_id.rfind("relocate_", 0) == 0) {
      robot.state = RobotMotionState::RELOCATING;
    } else if (ni->current_task_id.rfind("chain_retreat_", 0) == 0) {
      robot.state = RobotMotionState::CHAIN_STEP;
    } else if (ni->current_task_id.rfind("avoidance_", 0) == 0) {
      robot.state = RobotMotionState::YIELDING;
    } else if (ni->aligning_before_nav) {
      robot.state = RobotMotionState::ALIGNING;
    } else if (ni->has_active_goal || !ni->route.empty()) {
      robot.state = RobotMotionState::MOVING;
    } else if (!ni->current_task_id.empty()) {
      robot.state = RobotMotionState::ASSIGNED;
    } else if (robot.state == RobotMotionState::UNKNOWN) {
      robot.state = robot.online ? RobotMotionState::IDLE : RobotMotionState::OFFLINE;
    }

    if (ni->route.empty() || ni->route_index >= ni->route.size() ||
        ni->current_task_id.empty()) {
      continue;
    }

    std::string next_wp = ni->route[ni->route_index];
    std::string from_wp;
    if (ni->route_index > 0) {
      from_wp = ni->route[ni->route_index - 1];
    } else {
      from_wp = robot.current_wp;
    }
    std::string blocker = occupancy_->waypoint_blocker(rid, next_wp);
    if (blocker.empty() && !from_wp.empty() && from_wp != next_wp) {
      blocker = occupancy_->can_enter(rid, from_wp, next_wp);
    }
    if (blocker.empty()) {
      blocker = physical_waypoint_blocker(rid, next_wp);
    }
    if (!blocker.empty() && blocker != rid) {
      robot.state = RobotMotionState::WAITING_BLOCKER;
      WaitEdge edge;
      edge.waiter_robot_id = rid;
      edge.blocker_robot_id = blocker;
      edge.task_id = ni->current_task_id;
      edge.from_wp = from_wp;
      edge.to_wp = next_wp;
      edge.target_wp = robot.target_wp;
      edge.resource = from_wp.empty() ? next_wp : from_wp + "->" + next_wp;
      edge.active_navigation = true;
      add_wait_edge(edge);
    }
  }

  for (const auto & [_, wait] : task_waits_) {
    std::string resource;
    std::string blocker = resolve_wait_blocker(wait, resource);
    WaitEdge edge;
    edge.waiter_robot_id = wait.robot_id;
    edge.blocker_robot_id = blocker;
    edge.task_id = wait.task_id;
    edge.wait_state = wait.state;
    edge.from_wp = wait.from_wp;
    edge.to_wp = wait.to_wp;
    edge.target_wp = wait.target_wp;
    edge.resource = resource;
    add_wait_edge(edge);

    auto & robot = snapshot.robots[wait.robot_id];
    robot.robot_id = wait.robot_id;
    if (robot.task_id.empty()) robot.task_id = wait.task_id;
    if (robot.target_wp.empty()) robot.target_wp = wait.target_wp;
    if (robot.blocker_id.empty()) robot.blocker_id = blocker;
    if (robot.wait_resource.empty()) robot.wait_resource = resource;
    if (wait.state == TaskWaitState::WAIT_TARGET_CLEAR) {
      robot.state = RobotMotionState::WAITING_TARGET;
    } else if (wait.state == TaskWaitState::WAIT_ROUTE_CLEAR) {
      robot.state = RobotMotionState::WAITING_ROUTE;
    } else if (wait.state == TaskWaitState::SELF_RELOCATING) {
      robot.state = RobotMotionState::SELF_RELOCATING;
    } else if (wait.state == TaskWaitState::WAIT_BLOCKER_RELEASE) {
      robot.state = RobotMotionState::WAITING_BLOCKER;
    }
  }
  return snapshot;
}

bool FleetManagerNode::log_throttle_ok(const std::string & key, double min_interval_sec)
{
  const auto now = this->now();
  auto it = log_throttle_.find(key);
  if (it != log_throttle_.end() &&
      (now - it->second).seconds() < min_interval_sec) {
    return false;
  }
  log_throttle_[key] = now;
  // 简易 GC: 表过大时清掉过期条目,避免长跑泄漏
  if (log_throttle_.size() > 256) {
    for (auto i = log_throttle_.begin(); i != log_throttle_.end(); ) {
      if ((now - i->second).seconds() > 60.0) i = log_throttle_.erase(i);
      else ++i;
    }
  }
  return true;
}

void FleetManagerNode::deadlock_check()
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  const auto now = this->now();
  auto state = build_fleet_state_snapshot();
  const auto & block_graph = state.wait_graph;

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

  // 一般化 wait-cycle 断路器:
  // 只要环里至少有一条非活跃 (active_navigation == false) 的 wait edge,
  // 清掉这些 wait 条件并 requeue 即可破环。覆盖纯路线环、混合环
  // (TARGET / ROUTE / BLOCKER_RELEASE / SELF_RELOCATING) 等所有可被静态打破的死锁。
  std::set<std::string> cycle_robots(cycle.begin(), cycle.end());
  std::vector<WaitEdge> breakable_edges;
  std::set<std::string> active_in_cycle;
  for (const auto & rid : cycle) {
    const WaitEdge * picked = nullptr;
    bool any_active = false;
    for (const auto & edge : state.wait_edges) {
      if (edge.waiter_robot_id != rid) continue;
      if (!cycle_robots.count(edge.blocker_robot_id)) continue;
      if (edge.active_navigation) { any_active = true; continue; }
      picked = &edge;
      break;
    }
    if (picked) breakable_edges.push_back(*picked);
    else if (any_active) active_in_cycle.insert(rid);
  }

  if (!breakable_edges.empty()) {
    deadlock_break_count_++;
    bool pure_route = std::all_of(breakable_edges.begin(), breakable_edges.end(),
      [](const WaitEdge & e) { return e.wait_state == TaskWaitState::WAIT_ROUTE_CLEAR; });
    const char * tag = pure_route && active_in_cycle.empty() ?
      "deadlock.route_wait_cycle_break" : "deadlock.wait_cycle_break";
    std::ostringstream detail;
    detail << "wait cycle persisted for " << duration << "s, requeueing";
    for (const auto & edge : breakable_edges) {
      detail << " " << edge.task_id << "(" << edge.waiter_robot_id
             << "->" << edge.blocker_robot_id << ":";
      if (!edge.resource.empty()) detail << edge.resource;
      else if (!edge.target_wp.empty()) detail << "target=" << edge.target_wp;
      else detail << "?";
      detail << ")";
    }
    if (!active_in_cycle.empty()) {
      detail << " resetting_active=";
      bool first = true;
      for (const auto & rid : active_in_cycle) {
        if (!first) detail << ",";
        detail << rid;
        first = false;
      }
    }
    PersistLogger::log_warn(tag, "", "", detail.str(),
      __FILE__, __LINE__, __func__);

    // 错峰退避基数: 同环成员从 0 开始,每个加 stagger_step 秒,避免同一拍重抢同一资源
    constexpr double stagger_step = 2.0;
    int stagger_idx = 0;

    // 1. 清掉非活跃 wait 边并 requeue
    //    注: 死锁打破是系统级拓扑问题,不算 task 失败,因此用 mark_task_pending
    //    而非 mark_task_pending_retry,避免 retry_cycle_count_ 累计导致被动 task 被误杀。
    for (const auto & edge : breakable_edges) {
      clear_task_wait_condition(edge.task_id);
      scheduler_->mark_task_pending(edge.task_id);
      auto ni = get_or_create_nav(edge.waiter_robot_id);
      if (ni) {
        ni->retry_count = 0;
        ni->retry_after = now + rclcpp::Duration::from_seconds(
          stagger_step * static_cast<double>(stagger_idx));
      }
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(edge.task_id);
      if (!ti.task_id.empty()) task_pub_->publish(ti);
      ++stagger_idx;
    }

    // 2. mixed cycle 的关键修复:
    //    把环中的活跃等待者(中途 hop-blocked 的机器人)也复位,
    //    取消 nav goal、释放预留、把任务推回 pending,
    //    避免它们在下一拍立即把刚断开的环拉回来。
    for (const auto & rid : active_in_cycle) {
      auto ni_it = navs_.find(rid);
      if (ni_it == navs_.end() || !ni_it->second) continue;
      auto & ni = ni_it->second;
      std::string tid = ni->current_task_id;
      if (tid.empty()) continue;
      // 链参与者 / avoidance 等内部任务不动
      if (is_internal_task_id(tid)) continue;

      cancel_goals(ni);
      occupancy_->release_reservations(rid);
      clear_task_wait_condition(tid);
      scheduler_->mark_task_pending(tid);  // 同 P0-A: 死锁打破不增 cycle

      ni->current_task_id.clear();
      ni->route.clear();
      ni->route_index = 0;
      ni->retry_count = 0;
      ni->has_active_goal = false;
      ni->retry_after = now + rclcpp::Duration::from_seconds(
        stagger_step * static_cast<double>(stagger_idx));

      // 也从 waiting_for_ 里把这个机器人摘掉(任何依赖它的人都要重新评估)
      for (auto & [_, waiters] : waiting_for_) {
        (void)_;
        waiters.erase(rid);
      }

      PersistLogger::log_warn("deadlock.active_waiter_reset", rid, tid,
        "active waiter in mixed cycle reset to pending to avoid reformation",
        __FILE__, __LINE__, __func__);
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(tid);
      if (!ti.task_id.empty()) task_pub_->publish(ti);
      ++stagger_idx;
    }

    prev_cycle_key_.clear();
    cycle_first_seen_ = rclcpp::Time{};
    return;
  }

  // 打破死锁
  deadlock_break_count_++;
  PersistLogger::log_warn("deadlock.break", "", "",
    "deadlock persisted for " + std::to_string(duration) + "s, breaking",
    __FILE__, __LINE__, __func__);

  // 选最低优先级任务的底盘作为 victim
  std::string victim;
  int victim_pri = std::numeric_limits<int>::max();
  bool victim_has_active_nav = false;
  for (const auto & rid : cycle) {
    auto ni = navs_.find(rid);
    if (ni == navs_.end() || !ni->second) continue;
    bool has_active_nav = !ni->second->current_task_id.empty();
    auto ti = scheduler_->get_task_info(ni->second->current_task_id);
    int pri = ti.task_id.empty() ? 0 : ti.priority;
    if ((has_active_nav && !victim_has_active_nav) ||
        (has_active_nav == victim_has_active_nav &&
         (pri < victim_pri || (pri == victim_pri && victim.empty())))) {
      victim_pri = pri;
      victim = rid;
      victim_has_active_nav = has_active_nav;
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
      std::set<std::string> relocate_exclude;
      auto add_route_exclusion = [&](const std::string & rid, const std::string & target_wp) {
        if (rid.empty() || rid == victim || target_wp.empty()) return;
        auto rs = robots_.find(rid);
        if (rs == robots_.end()) return;
        std::string start_wp = rs->second.current_waypoint;
        if (start_wp.empty()) start_wp = traffic_->find_nearest_waypoint(rs->second.current_pose);
        if (start_wp.empty()) return;
        auto path = traffic_->find_path(start_wp, target_wp);
        for (const auto & wp : path) relocate_exclude.insert(wp);
      };
      for (const auto & [rid, ri] : navs_) {
        if (rid == victim || !ri) continue;
        if (!ri->route.empty()) {
          for (size_t i = ri->route_index; i < ri->route.size(); ++i) {
            relocate_exclude.insert(ri->route[i]);
          }
        }
        if (!ri->current_task_id.empty()) {
          auto ti = scheduler_->get_task_info(ri->current_task_id);
          if (!ti.task_id.empty()) add_route_exclusion(rid, ti.waypoint_id);
        }
      }
      for (const auto & [_, wait] : task_waits_) {
        if (wait.robot_id == victim) continue;
        if (!wait.from_wp.empty()) relocate_exclude.insert(wait.from_wp);
        if (!wait.to_wp.empty()) relocate_exclude.insert(wait.to_wp);
        add_route_exclusion(wait.robot_id, wait.target_wp);
      }
      for (const auto & t : scheduler_->get_all_tasks()) {
        if (t.task_id.empty() || t.assigned_robot_id.empty() ||
            t.assigned_robot_id == victim || t.waypoint_id.empty()) {
          continue;
        }
        if (t.status == "completed" || t.status == "failed" ||
            t.status == "cancelled") {
          continue;
        }
        relocate_exclude.insert(t.waypoint_id);
        add_route_exclusion(t.assigned_robot_id, t.waypoint_id);
      }

      std::string relocate_wp;
      std::vector<std::string> relocate_path;
      while (true) {
        std::string candidate = find_safe_free_waypoint(cur_wp, relocate_exclude, victim);
        if (candidate.empty() || candidate == cur_wp) break;
        auto path = traffic_->find_path(cur_wp, candidate);
        if (path.size() < 2) {
          relocate_exclude.insert(candidate);
          continue;
        }
        std::string first_hop = path[1];
        std::string blocker = occupancy_->can_enter(victim, cur_wp, first_hop);
        if (!blocker.empty() && blocker != victim) {
          relocate_exclude.insert(candidate);
          continue;
        }
        if (!occupancy_->reserve_next(victim, cur_wp, first_hop)) {
          relocate_exclude.insert(candidate);
          continue;
        }
        relocate_wp = candidate;
        relocate_path = std::move(path);
        break;
      }

      if (!relocate_wp.empty() && !relocate_path.empty()) {
        PersistLogger::log_info("deadlock.relocate_victim", victim, tid,
          "relocating victim from " + cur_wp + " to " + relocate_wp,
          __FILE__, __LINE__, __func__);
        ni->second->route = relocate_path;
        ni->second->route_index = 0;
        ni->second->route_alignment_done = false;
        ni->second->current_task_id = "relocate_" + victim;
        ni->second->retry_count = 0;
        ni->second->retry_after = rclcpp::Time{};
        navigate_to_next_waypoint(victim);
      } else {
        PersistLogger::log_warn("deadlock.relocate_unavailable", victim, tid,
          "no safe relocation waypoint from " + cur_wp,
          __FILE__, __LINE__, __func__);
      }
    }
  }

  if (!tid.empty()) {
    scheduler_->mark_task_pending_retry(tid);
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
  auto snapshot = build_fleet_state_snapshot();

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
  std::ostringstream wait_oss;
  bool first_wait = true;
  for (const auto & edge : snapshot.wait_edges) {
    if (!first_wait) wait_oss << ",";
    first_wait = false;
    wait_oss << edge.waiter_robot_id << ">" << edge.blocker_robot_id;
    if (!edge.resource.empty()) wait_oss << ":" << edge.resource;
    if (!edge.task_id.empty()) wait_oss << ":" << edge.task_id;
  }
  oss << "robots_online=" << online
      << " robots_offline=" << offline
      << " tasks_pending=" << pending
      << " tasks_active=" << active
      << " tasks_completed=" << completed
      << " tasks_failed=" << failed
      << " wait_edges=" << snapshot.wait_edges.size()
      << " wait_graph=" << (first_wait ? "-" : wait_oss.str())
      << " deadlock_breaks=" << deadlock_break_count_;
  msg.data = oss.str();
  metrics_pub_->publish(msg);
}

std::string FleetManagerNode::robot_motion_state_name(RobotMotionState state) const
{
  switch (state) {
    case RobotMotionState::UNKNOWN: return "unknown";
    case RobotMotionState::OFFLINE: return "offline";
    case RobotMotionState::IDLE: return "idle";
    case RobotMotionState::ASSIGNED: return "assigned";
    case RobotMotionState::ALIGNING: return "aligning";
    case RobotMotionState::MOVING: return "moving";
    case RobotMotionState::WAITING_BLOCKER: return "waiting_blocker";
    case RobotMotionState::WAITING_TARGET: return "waiting_target";
    case RobotMotionState::WAITING_ROUTE: return "waiting_route";
    case RobotMotionState::SELF_RELOCATING: return "self_relocating";
    case RobotMotionState::YIELDING: return "yielding";
    case RobotMotionState::RELOCATING: return "relocating";
    case RobotMotionState::CHAIN_STEP: return "chain_step";
    case RobotMotionState::EXECUTING: return "executing";
    case RobotMotionState::CONFLICT: return "conflict";
    case RobotMotionState::GHOST: return "ghost";
  }
  return "unknown";
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
  std::map<std::string, std::string> wait_blocker_updates;
  for (const auto & [task_id, wait] : task_waits_) {
    if (wait.blocker_id != blocker_id) continue;
    std::string resource;
    std::string actual_blocker = resolve_wait_blocker(wait, resource);
    bool ready = actual_blocker.empty() ||
      ((wait.state == TaskWaitState::WAIT_BLOCKER_RELEASE ||
        wait.state == TaskWaitState::SELF_RELOCATING) &&
       actual_blocker != wait.blocker_id);
    if (ready) {
      ready_tasks.push_back(task_id);
    } else {
      const std::string next_blocker = actual_blocker.empty() ? blocker_id : actual_blocker;
      waiting_for_[next_blocker].insert(wait.robot_id);
      if (next_blocker != blocker_id) wait_blocker_updates[task_id] = next_blocker;
    }
  }
  for (const auto & [task_id, next_blocker] : wait_blocker_updates) {
    auto wait_it = task_waits_.find(task_id);
    if (wait_it != task_waits_.end()) wait_it->second.blocker_id = next_blocker;
  }
  for (const auto & task_id : ready_tasks) {
    auto ready_it = task_waits_.find(task_id);
    if (ready_it == task_waits_.end()) continue;
    auto wait = ready_it->second;
    clear_task_wait_condition(task_id);
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
      if (blocker.empty()) {
        blocker = physical_waypoint_blocker(waiter_id, to);
      }
      if (!blocker.empty()) {
        auto blocker_ni = get_or_create_nav(blocker);
        bool blocker_idle = blocker_ni &&
          !blocker_ni->has_active_goal &&
          blocker_ni->current_task_id.empty() &&
          blocker_ni->route.empty() &&
          !blocker_ni->chassis_task_sent;
        if (is_internal_task_id(ni->current_task_id) && blocker_idle) {
          ni->retry_after = rclcpp::Time{};
          PersistLogger::log_info("sched.waiter_retry_idle_blocker", waiter_id, ni->current_task_id,
            "next hop " + from + "->" + to + " still blocked by idle " + blocker,
            __FILE__, __LINE__, __func__);
          navigate_to_next_waypoint(waiter_id);
          continue;
        }
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

std::vector<std::string> FleetManagerNode::plan_route_for_task(
  const std::string & robot_id,
  const std::string & target_wp) const
{
  std::string start_wp;
  auto st = robots_.find(robot_id);
  if (st != robots_.end()) {
    start_wp = st->second.current_waypoint;
    if (start_wp.empty()) start_wp = traffic_->find_nearest_waypoint(st->second.current_pose);
  }

  std::vector<std::string> path;
  if (!start_wp.empty() && start_wp != target_wp) {
    path = traffic_->find_path(start_wp, target_wp);
  }
  if (path.empty() && !start_wp.empty() && start_wp != target_wp) {
    path = {start_wp, target_wp};
  }
  if (path.empty()) path.push_back(target_wp);

  std::vector<std::string> clean;
  for (const auto & wp : path) {
    if (clean.empty() || clean.back() != wp) clean.push_back(wp);
  }
  return clean;
}

std::string FleetManagerNode::physical_waypoint_blocker(
  const std::string & robot_id,
  const std::string & wp_id) const
{
  if (wp_id.empty()) return "";
  const auto wp_pose = traffic_->get_waypoint_pose(wp_id);
  const double radius = std::max(waypoint_radius_, traffic_->get_waypoint_radius(wp_id));
  for (const auto & [rid, st] : robots_) {
    if (rid == robot_id) continue;
    if (st.connection_status != "online") continue;
    if (std::abs(st.current_pose.position.x) < 1e-6 &&
        std::abs(st.current_pose.position.y) < 1e-6) {
      continue;
    }
    const double dx = st.current_pose.position.x - wp_pose.position.x;
    const double dy = st.current_pose.position.y - wp_pose.position.y;
    if (std::hypot(dx, dy) <= radius) return rid;
  }
  return "";
}

std::string FleetManagerNode::find_active_route_conflict(
  const std::string & robot_id,
  const std::string & task_id,
  const std::vector<std::string> & path,
  std::string & conflict_task_id,
  std::string & conflict_resource,
  bool include_waiting_intents) const
{
  conflict_task_id.clear();
  conflict_resource.clear();
  if (path.empty() || is_internal_task_id(task_id)) return {};

  std::set<std::string> candidate_wps;
  const size_t first_candidate_wp = path.size() > 1 ? 1 : 0;
  for (size_t i = first_candidate_wp; i < path.size(); ++i) {
    if (!path[i].empty()) candidate_wps.insert(path[i]);
  }

  std::set<std::pair<std::string, std::string>> candidate_edges;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    if (!path[i].empty() && !path[i + 1].empty() && path[i] != path[i + 1]) {
      candidate_edges.insert({path[i], path[i + 1]});
    }
  }

  for (const auto & [active_robot, ni] : navs_) {
    if (active_robot == robot_id || !ni) continue;
    if (ni->current_task_id.empty() || ni->current_task_id == task_id) continue;
    if (is_internal_task_id(ni->current_task_id)) continue;
    if (ni->route.empty() || ni->route_index >= ni->route.size()) continue;

    const size_t active_begin = ni->route_index > 0 ? ni->route_index - 1 : ni->route_index;
    for (const auto & [from, to] : candidate_edges) {
      for (size_t i = active_begin; i + 1 < ni->route.size(); ++i) {
        if (ni->route[i] == to && ni->route[i + 1] == from) {
          conflict_task_id = ni->current_task_id;
          conflict_resource = from + "->" + to;
          return active_robot;
        }
      }
    }

    std::set<std::string> active_wps;
    for (size_t i = active_begin; i < ni->route.size(); ++i) {
      if (!ni->route[i].empty()) active_wps.insert(ni->route[i]);
    }
    for (const auto & wp : candidate_wps) {
      if (active_wps.count(wp)) {
        conflict_task_id = ni->current_task_id;
        conflict_resource = wp;
        return active_robot;
      }
    }
  }
  if (include_waiting_intents) {
    for (const auto & [wait_task_id, wait] : task_waits_) {
      if (wait_task_id == task_id || wait.robot_id == robot_id ||
          wait.blocker_id == robot_id || wait.target_wp.empty() ||
          is_internal_task_id(wait_task_id)) {
        continue;
      }
      auto wait_path = plan_route_for_task(wait.robot_id, wait.target_wp);
      if (wait_path.empty()) continue;

      std::set<std::pair<std::string, std::string>> wait_edges;
      for (size_t i = 0; i + 1 < wait_path.size(); ++i) {
        if (!wait_path[i].empty() && !wait_path[i + 1].empty() &&
            wait_path[i] != wait_path[i + 1]) {
          wait_edges.insert({wait_path[i], wait_path[i + 1]});
        }
      }
      for (const auto & [from, to] : candidate_edges) {
        if (wait_edges.count({to, from})) {
          conflict_task_id = wait_task_id;
          conflict_resource = from + "->" + to;
          return wait.robot_id;
        }
      }

      std::set<std::string> wait_wps;
      const size_t wait_first_wp = wait_path.size() > 1 ? 1 : 0;
      for (size_t i = wait_first_wp; i < wait_path.size(); ++i) {
        if (!wait_path[i].empty()) wait_wps.insert(wait_path[i]);
      }
      for (const auto & wp : candidate_wps) {
        if (wait_wps.count(wp)) {
          conflict_task_id = wait_task_id;
          conflict_resource = wp;
          return wait.robot_id;
        }
      }
    }
  }
  return {};
}

}  // namespace fleet_manager
