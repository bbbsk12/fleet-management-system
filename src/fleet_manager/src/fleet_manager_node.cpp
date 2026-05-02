#include "fleet_manager/fleet_manager_node.hpp"
#include "fleet_manager/persist_logger.hpp"
#include "fleet_manager/internal/fleet_manager_node_internal.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <algorithm>
#include <cmath>

namespace fleet_manager
{

// ==================== Lifecycle ====================

FleetManagerNode::FleetManagerNode()
: Node("fleet_manager")
{
  // subscriptions
  fleet_sub_ = this->create_subscription<fleet_msgs::msg::FleetStatus>(
    "/fleet_monitor/fleet_status", 10,
    std::bind(&FleetManagerNode::fleet_status_callback, this, std::placeholders::_1));

  // publishers
  traffic_pub_ = this->create_publisher<fleet_msgs::msg::FleetStatus>(
    "/fleet_manager/fleet_status_traffic", 10);
  task_pub_ = this->create_publisher<fleet_msgs::msg::TaskInfo>("~/task_status", 10);

  // services
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

  // core modules
  scheduler_  = std::make_unique<TaskScheduler>(this);
  traffic_    = std::make_unique<TrafficManager>(this);
  occupancy_  = std::make_unique<OccupancyManager>(this);

  // params
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

  waypoint_radius_  = this->get_parameter("waypoint_acceptance_radius").as_double();
  segment_lateral_  = this->get_parameter("traffic_segment_lateral_max").as_double();
  sched_interval_   = std::max(0.1, this->get_parameter("scheduler_interval_sec").as_double());
  retry_base_       = std::max(0.5, this->get_parameter("nav_retry_base_sec").as_double());
  retry_max_        = this->get_parameter("nav_retry_max").as_int();
  nav_stuck_timeout_    = this->get_parameter("nav_stuck_timeout_sec").as_double();
  nav_absolute_timeout_  = this->get_parameter("nav_absolute_timeout_sec").as_double();
  chassis_hs_timeout_   = this->get_parameter("chassis_handshake_timeout_sec").as_double();
  chassis_exec_timeout_ = this->get_parameter("chassis_exec_timeout_sec").as_double();
  chassis_max_retries_  = this->get_parameter("chassis_max_retries").as_int();
  monitor_stale_timeout_ = this->get_parameter("monitor_fleet_stale_timeout_sec").as_double();

  {
    bool en = this->get_parameter("persist_log_enabled").as_bool();
    std::string dir = this->get_parameter("persist_log_dir").as_string();
    bool vb = this->get_parameter("persist_log_verbose_info").as_bool();
    PersistLogger::init(en, dir, "fleet_manager", vb);
  }

  std::string map_file = this->get_parameter("traffic_map_file").as_string();
  if (!map_file.empty() && traffic_->load_map(map_file)) {
    RCLCPP_INFO(this->get_logger(), "Loaded traffic map: %s", map_file.c_str());
    occupancy_->set_topology(
      traffic_->get_adjacency_map(),
      [this](const std::string & id) { return traffic_->get_waypoint_pose(id); },
      [this](const std::string & id) { return traffic_->get_waypoint_radius(id); });
    traffic_->validate_waypoint_spacing(waypoint_radius_ * 2.0);
  }

  // timers: control (slow, 500ms) + fast (200ms)
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

// ==================== Timers ====================

void FleetManagerNode::control_timer_callback()
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  ++tick_;

  // update occupancy from live robot positions
  for (auto & [rid, st] : robots_) {
    if (st.connection_status != "online") continue;
    if (std::abs(st.current_pose.position.x) < 1e-6 &&
        std::abs(st.current_pose.position.y) < 1e-6) {
      // zero pose = lost localization → release locks to prevent ghost blocking
      occupancy_->release_locks(rid);
      occupancy_->release_reservations(rid);
      st.location_type = "unknown";
      st.current_waypoint = "";
      st.current_segment = "";
      continue;
    }

    auto loc = occupancy_->update_location(
      rid, st.current_pose, waypoint_radius_, segment_lateral_);

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

  // schedule tick every sched_interval_
  static int sched_phase = 0;
  int period = std::max(1, static_cast<int>(sched_interval_ / 0.5));
  if (++sched_phase >= period) {
    sched_phase = 0;
    schedule_tick();
  }

  publish_traffic_fleet_status();
}

void FleetManagerNode::fast_timer_callback()
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  led_timer_callback();

  // retry navigation for waiting robots
  for (auto & [rid, ni] : navs_) {
    if (!ni || ni->has_active_goal) continue;
    if (ni->current_task_id.empty()) continue;
    if (ni->route.empty()) continue;
    if (ni->retry_after.nanoseconds() > 0 && this->now() < ni->retry_after) continue;

    navigate_to_next_waypoint(rid);
  }
}

// ==================== Fleet Status ====================

void FleetManagerNode::fleet_status_callback(
  const fleet_msgs::msg::FleetStatus::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  last_fleet_ = *msg;
  has_fleet_ = true;
  last_fleet_time_ = this->now();

  std::set<std::string> prev_online;
  for (const auto & [rid, st] : robots_)
    if (st.connection_status == "online") prev_online.insert(rid);

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

  // mark vanished robots offline
  for (auto & [rid, st] : robots_) {
    if (reported.count(rid)) continue;
    if (st.connection_status == "online") st.connection_status = "offline";
  }

  // online→offline: release reservations, keep position lock (ghost guard)
  for (const auto & rid : prev_online) {
    auto it = robots_.find(rid);
    if (it != robots_.end() && it->second.connection_status != "online") {
      occupancy_->release_reservations(rid);
      auto ni = get_or_create_nav(rid);
      if (ni) { cancel_goals(ni); stop_robot(rid, 10); }
      PersistLogger::log_warn("robot.offline", rid, "",
        "positions locked as ghost guard", __FILE__, __LINE__, __func__);
    }
  }
  // offline→online: clear ghost locks, let update_location re-establish
  for (const auto & [rid, st] : robots_) {
    auto prev = prev_online.find(rid);
    if (prev == prev_online.end() && st.connection_status == "online") {
      occupancy_->release_locks(rid);
      occupancy_->release_reservations(rid);
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

// ==================== Scheduling ====================

void FleetManagerNode::schedule_tick()
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  occupancy_->expire_stale_reservations();
  scheduler_->purge_finished(200);

  // recover orphaned tasks
  const auto now = this->now();
  for (const auto & t : scheduler_->get_all_tasks()) {
    if (t.status != "in_progress") continue;
    if (t.assigned_robot_id.empty()) continue;

    auto ni = get_or_create_nav(t.assigned_robot_id);
    if (!ni) continue;

    bool idle = !ni->has_active_goal && ni->route.empty();
    if (ni->current_task_id == t.task_id && idle) {
      // check if just arrived at final waypoint (chassis might be executing)
      if (t.status == "in_progress" && ni->chassis_task_sent) continue;
      // stale binding: recover
      PersistLogger::log_warn("sched.orphan", t.assigned_robot_id, t.task_id,
        "recovering orphaned task", __FILE__, __LINE__, __func__);
      scheduler_->mark_task_pending(t.task_id);
      fleet_msgs::msg::TaskInfo ti = scheduler_->get_task_info(t.task_id);
      if (!ti.task_id.empty()) task_pub_->publish(ti);
    }
  }

  assign_pending_tasks();
}

void FleetManagerNode::assign_pending_tasks()
{
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  const auto now = this->now();
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

  // build online robot list
  std::vector<fleet_msgs::msg::RobotStatus> online;
  for (const auto & [rid, st] : robots_) {
    auto ni = get_or_create_nav(rid);
    if (!ni) continue;

    if (st.connection_status != "online") continue;
    if (std::abs(st.current_pose.position.x) < 1e-6 &&
        std::abs(st.current_pose.position.y) < 1e-6) continue;

    // skip busy robots (navigating or executing)
    if (ni->has_active_goal || !ni->route.empty() ||
        !ni->current_task_id.empty() || ni->chassis_task_sent) continue;

    auto st_copy = st;
    st_copy.connection_status = "online";
    online.push_back(st_copy);
  }

  if (online.empty()) return;

  auto wp_poses = traffic_->get_all_waypoint_poses();
  auto assigned = scheduler_->assign_tasks_batch(online, wp_poses);

  for (auto & t : assigned) {
    if (t.task_id.empty()) continue;

    // safety check: robot must be truly idle
    auto ni = get_or_create_nav(t.assigned_robot_id);
    if (ni && (ni->has_active_goal || !ni->route.empty() ||
               !ni->current_task_id.empty() || ni->chassis_task_sent)) {
      scheduler_->mark_task_pending(t.task_id);
      continue;
    }

    PersistLogger::log_info("sched.assign", t.assigned_robot_id, t.task_id,
      "assigned to wp=" + t.waypoint_id, __FILE__, __LINE__, __func__);
    start_navigation(t.assigned_robot_id, t.waypoint_id, t.task_id);
  }
}

}  // namespace fleet_manager
