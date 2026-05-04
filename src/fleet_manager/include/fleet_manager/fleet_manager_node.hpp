#ifndef FLEET_MANAGER__FLEET_MANAGER_NODE_HPP_
#define FLEET_MANAGER__FLEET_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fleet_msgs/msg/fleet_status.hpp>
#include <fleet_msgs/msg/robot_status.hpp>
#include <fleet_msgs/msg/task_info.hpp>
#include <fleet_msgs/msg/task_cmd.hpp>
#include <fleet_msgs/msg/task_fb.hpp>
#include <fleet_msgs/msg/led_task.hpp>
#include <fleet_msgs/msg/led_status.hpp>
#include <fleet_msgs/srv/submit_task.hpp>
#include <fleet_msgs/srv/cancel_task.hpp>
#include <fleet_msgs/srv/get_robot_status.hpp>
#include <fleet_msgs/srv/load_traffic_map.hpp>
#include <fleet_msgs/srv/save_traffic_map.hpp>
#include <fleet_msgs/srv/remove_robot.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <mutex>
#include "task_scheduler.hpp"
#include "traffic_manager.hpp"
#include "occupancy_manager.hpp"

namespace fleet_manager
{

using NavigateToPose       = nav2_msgs::action::NavigateToPose;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using FollowWaypoints      = nav2_msgs::action::FollowWaypoints;
using GoalHandleNavigate   = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct RobotNavInfo
{
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client;

  std::string current_task_id;
  std::vector<std::string> route;
  size_t route_index{0};
  bool has_active_goal{false};

  GoalHandleNavigate::SharedPtr goal_handle;

  int retry_count{0};
  rclcpp::Time retry_after{};
  rclcpp::Time nav_since{};
  rclcpp::Time nav_last_activity{};
  rclcpp::Time recent_cancel_until{};
  uint64_t nav_seq{0};

  // chassis task
  rclcpp::Publisher<fleet_msgs::msg::TaskCmd>::SharedPtr  task_cmd_pub;
  rclcpp::Subscription<fleet_msgs::msg::TaskFb>::SharedPtr task_fb_sub;
  bool   chassis_task_sent{false};
  bool   chassis_handshake_ok{false};
  bool   chassis_acked{false};
  rclcpp::Time chassis_hs_deadline{};
  rclcpp::Time chassis_exec_deadline{};
  int    chassis_retries{0};
  uint8_t  pending_task_type{1};
  uint32_t pending_site_code{0};
  uint16_t pending_wp_num{0};
  uint64_t pending_task_num{0};

  // LED
  rclcpp::Publisher<fleet_msgs::msg::LEDTask>::SharedPtr  led_pub;
  rclcpp::Subscription<fleet_msgs::msg::LEDStatus>::SharedPtr led_sub;
  uint8_t last_led_state{0xFF};
  uint8_t chassis_led{0xFF};
  bool    led_received{false};
};

// ── Chain retreat structures ───────────────────────

struct RetreatChainStep
{
  std::string robot_id;
  std::string target_wp;   // single-hop destination
};

struct ChainRetreatPlan
{
  std::string original_requester;
  std::string original_target;
  std::string original_task_id;
  // Saved state for participants that had tasks before the chain
  std::map<std::string, std::string> saved_task_ids;    // robot_id → original task_id
  std::map<std::string, std::string> saved_targets;     // robot_id → original target_wp
  std::vector<RetreatChainStep> steps;
  size_t current_step{0};
  bool active{false};
  rclcpp::Time started_at;
  int step_retry_count{0};
};

class FleetManagerNode : public rclcpp::Node
{
public:
  FleetManagerNode();
  ~FleetManagerNode() override;

private:
  // ── Timers ──────────────────────────────────────────
  void control_timer_callback();
  void fast_timer_callback();

  // ── States ──────────────────────────────────────────
  void fleet_status_callback(const fleet_msgs::msg::FleetStatus::SharedPtr msg);

  // ── Services ────────────────────────────────────────
  void handle_submit_task(
    const std::shared_ptr<fleet_msgs::srv::SubmitTask::Request> req,
    std::shared_ptr<fleet_msgs::srv::SubmitTask::Response> res);
  void handle_cancel_task(
    const std::shared_ptr<fleet_msgs::srv::CancelTask::Request> req,
    std::shared_ptr<fleet_msgs::srv::CancelTask::Response> res);
  void handle_get_robot_status(
    const std::shared_ptr<fleet_msgs::srv::GetRobotStatus::Request> req,
    std::shared_ptr<fleet_msgs::srv::GetRobotStatus::Response> res);
  void handle_load_traffic_map(
    const std::shared_ptr<fleet_msgs::srv::LoadTrafficMap::Request> req,
    std::shared_ptr<fleet_msgs::srv::LoadTrafficMap::Response> res);
  void handle_save_traffic_map(
    const std::shared_ptr<fleet_msgs::srv::SaveTrafficMap::Request> req,
    std::shared_ptr<fleet_msgs::srv::SaveTrafficMap::Response> res);
  void handle_remove_robot(
    const std::shared_ptr<fleet_msgs::srv::RemoveRobot::Request> req,
    std::shared_ptr<fleet_msgs::srv::RemoveRobot::Response> res);

  void publish_traffic_fleet_status();
  void publish_metrics();

  // ── Scheduling ──────────────────────────────────────
  void schedule_tick();
  void assign_pending_tasks();
  void deadlock_check();

  // ── Navigation ──────────────────────────────────────
  bool start_navigation(const std::string & robot_id,
                        const std::string & target_wp,
                        const std::string & task_id);
  void navigate_to_next_waypoint(const std::string & robot_id);
  void navigate_to_waypoint(const std::string & robot_id,
                            const std::string & wp_id,
                            const std::string & task_id,
                            bool is_final);
  void on_nav_succeeded(const std::string & robot_id,
                        const std::string & task_id);
  void check_arrivals();

  // ── Chain retreat coordination ─────────────────────
  bool is_robot_idle(const std::string & robot_id) const;
  bool is_robot_stationary(const std::string & robot_id) const;
  bool is_mutual_block(const std::string & blocker, const std::string & blocker_wp,
                       const std::string & requester, const std::string & requester_wp) const;
  bool try_build_retreat_chain(const std::string & requester, const std::string & from_wp,
                               const std::string & to_wp, const std::string & blocker,
                               const std::set<std::string> & blocked_set, int depth);
  void execute_chain_step();
  void on_chain_step_complete(const std::string & robot_id, bool nav_success);
  void abort_chain(const std::string & reason);

  // ── Chassis ─────────────────────────────────────────
  void send_chassis_cmd(const std::string & robot_id,
                        const std::string & task_id,
                        uint16_t wp_num, uint8_t type, uint32_t site);
  void send_chassis_ack(const std::string & robot_id, uint64_t task_num);
  void chassis_fb_callback(const std::string & robot_id,
                           const fleet_msgs::msg::TaskFb::SharedPtr msg);
  void chassis_timeout_check();
  bool is_robot_executing(const std::string & robot_id) const;
  void finalize_task_completion(const std::string & robot_id,
                                const std::string & task_id);
  uint16_t wp_to_u16(const std::string & wp_id) const;

  // ── LED ─────────────────────────────────────────────
  void led_timer_callback();
  uint8_t determine_led_state(const std::string & robot_id) const;
  void led_status_callback(const std::string & robot_id,
                           const fleet_msgs::msg::LEDStatus::SharedPtr msg);

  // ── Helpers ─────────────────────────────────────────
  void cancel_goals(const std::shared_ptr<RobotNavInfo> & ni);
  void cancel_all_goals();
  void stop_robot(const std::string & robot_id, int burst = 10);
  void stop_all();
  std::shared_ptr<RobotNavInfo> get_or_create_nav(const std::string & robot_id);
  double normalize_angle(double a) const;
  double get_yaw(const geometry_msgs::msg::Quaternion & q) const;
  std::string join_route(const std::vector<std::string> & wps) const;

  // ── ROS I/O ─────────────────────────────────────────
  rclcpp::Subscription<fleet_msgs::msg::FleetStatus>::SharedPtr fleet_sub_;
  rclcpp::Publisher<fleet_msgs::msg::FleetStatus>::SharedPtr  traffic_pub_;
  rclcpp::Publisher<fleet_msgs::msg::TaskInfo>::SharedPtr      task_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           alert_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           metrics_pub_;

  rclcpp::Service<fleet_msgs::srv::SubmitTask>::SharedPtr     submit_srv_;
  rclcpp::Service<fleet_msgs::srv::CancelTask>::SharedPtr     cancel_srv_;
  rclcpp::Service<fleet_msgs::srv::GetRobotStatus>::SharedPtr status_srv_;
  rclcpp::Service<fleet_msgs::srv::LoadTrafficMap>::SharedPtr load_srv_;
  rclcpp::Service<fleet_msgs::srv::SaveTrafficMap>::SharedPtr save_srv_;
  rclcpp::Service<fleet_msgs::srv::RemoveRobot>::SharedPtr    remove_srv_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr fast_timer_;

  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> vel_pubs_;

  // ── Core modules ────────────────────────────────────
  std::unique_ptr<TaskScheduler>    scheduler_;
  std::unique_ptr<TrafficManager>   traffic_;
  std::unique_ptr<OccupancyManager> occupancy_;

  // ── State ───────────────────────────────────────────
  mutable std::recursive_mutex mtx_;
  std::map<std::string, fleet_msgs::msg::RobotStatus> robots_;
  std::map<std::string, std::shared_ptr<RobotNavInfo>> navs_;
  std::set<std::string> removed_;

  fleet_msgs::msg::FleetStatus last_fleet_;
  bool has_fleet_{false};
  rclcpp::Time last_fleet_time_{};
  uint64_t tick_{0};

  // ── Chain retreat state ─────────────────────────────
  ChainRetreatPlan chain_plan_;

  // ── Deadlock detection state ────────────────────────────
  std::string prev_cycle_key_;    // serialized key of last detected cycle
  rclcpp::Time cycle_first_seen_; // when the current cycle was first detected
  uint64_t deadlock_break_count_{0}; // total deadlocks broken
  rclcpp::Time last_metrics_time_{}; // last metrics publication time

  // ── Params ──────────────────────────────────────────
  double waypoint_radius_{0.5};
  double segment_lateral_{1.2};
  double sched_interval_{1.0};
  double retry_base_{1.0};
  int    retry_max_{5};
  double nav_stuck_timeout_{20.0};
  double nav_absolute_timeout_{45.0};
  double chassis_hs_timeout_{5.0};
  double chassis_exec_timeout_{30.0};
  int    chassis_max_retries_{3};
  double monitor_stale_timeout_{4.0};
  double ghost_lock_ttl_{120.0};
  double deadlock_timeout_{10.0};
  int    max_task_retry_cycles_{5};
};

}  // namespace fleet_manager

#endif  // FLEET_MANAGER__FLEET_MANAGER_NODE_HPP_
