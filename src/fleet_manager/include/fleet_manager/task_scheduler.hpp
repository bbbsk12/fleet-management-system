#ifndef FLEET_MANAGER__TASK_SCHEDULER_HPP_
#define FLEET_MANAGER__TASK_SCHEDULER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <fleet_msgs/msg/task_info.hpp>
#include <fleet_msgs/msg/robot_status.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <queue>
#include <vector>
#include <map>
#include <set>
#include <mutex>

namespace fleet_manager
{

struct TaskCompare
{
  bool operator()(const fleet_msgs::msg::TaskInfo & a,
                  const fleet_msgs::msg::TaskInfo & b) const
  {
    if (a.priority != b.priority) return a.priority < b.priority;
    if (a.created_at.sec != b.created_at.sec) return a.created_at.sec > b.created_at.sec;
    if (a.created_at.nanosec != b.created_at.nanosec) return a.created_at.nanosec > b.created_at.nanosec;
    return a.task_id > b.task_id;
  }
};

class TaskScheduler
{
public:
  explicit TaskScheduler(rclcpp::Node * node);

  std::string submit_task(
    const std::string & waypoint_id,
    int priority = 0,
    const std::string & robot_id = "",
    uint8_t task_type = 1,
    uint32_t site_code = 0);

  std::vector<fleet_msgs::msg::TaskInfo> assign_tasks_batch(
    const std::vector<fleet_msgs::msg::RobotStatus> & robots,
    const std::map<std::string, geometry_msgs::msg::Pose> & waypoint_poses);

  void mark_task_navigating(const std::string & task_id);
  void mark_task_executing(const std::string & task_id);
  void mark_task_waiting(const std::string & task_id);
  void mark_task_pending(const std::string & task_id);
  void mark_task_pending_preserve(const std::string & task_id);

  void complete_task(const std::string & task_id);
  void fail_task(const std::string & task_id, const std::string & reason);
  void cancel_task(const std::string & task_id);

  /// Check if re-queuing this task again would exceed the retry cycle limit
  bool would_exceed_retry_cycles(const std::string & task_id, int max_cycles) const;
  int  retry_cycle_count(const std::string & task_id) const;

  uint8_t get_task_type(const std::string & task_id) const;
  fleet_msgs::msg::TaskInfo get_task_info(const std::string & task_id) const;
  std::vector<fleet_msgs::msg::TaskInfo> get_all_tasks() const;

  size_t pending_count() const;
  void repair_queue();
  void purge_finished(size_t max_keep = 200);

private:
  void remove_from_queue(const std::string & task_id);
  std::string generate_id();

  rclcpp::Node * node_;

  std::priority_queue<fleet_msgs::msg::TaskInfo,
                      std::vector<fleet_msgs::msg::TaskInfo>,
                      TaskCompare> queue_;

  std::map<std::string, fleet_msgs::msg::TaskInfo> all_;
  std::set<std::string> fixed_;  // tasks with pre-specified robot_id
  std::map<std::string, geometry_msgs::msg::Pose> waypoint_poses_;

  int counter_{0};
  std::map<std::string, int> fcfc_skip_count_;  // FCFS gate consecutive block counter
  std::map<std::string, int> retry_cycle_count_; // re-queue count for mark_task_pending_preserve
  static constexpr int kFcfcGateMaxSkips = 30;   // ~30s at default 1s interval
  mutable std::recursive_mutex mutex_;
};

}  // namespace fleet_manager

#endif  // FLEET_MANAGER__TASK_SCHEDULER_HPP_
