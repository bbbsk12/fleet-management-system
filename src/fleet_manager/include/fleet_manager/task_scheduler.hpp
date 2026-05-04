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

// ============================================================================
// 任务优先级比较器 — 用于 std::priority_queue 的最大堆排序
// 排序规则: 优先级数大 → 创建时间早 → task_id 小
// ============================================================================

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

// ============================================================================
// TaskScheduler — 任务队列与贪心调度
//
// 提供任务的完整生命周期管理:
//   提交 → 排队 → 分配(最近底盘贪心) → 导航中 → 执行中 → 完成/失败/取消
//
// 状态机: pending → assigned → in_progress → executing → completed
//                                ↘ waiting_fleet (退避重试)
// 终态: completed / failed / cancelled
// ============================================================================

class TaskScheduler
{
public:
  explicit TaskScheduler(rclcpp::Node * node);

  /// 提交任务到队列。robot_id 非空则为固定分配(fixed)，不参与最近底盘贪心
  std::string submit_task(
    const std::string & waypoint_id,
    int priority = 0,
    const std::string & robot_id = "",
    uint8_t task_type = 1,
    uint32_t site_code = 0);

  /// 批量分配: 将 pending 任务按优先级分配给空闲底盘(最近距离贪心 + FCFS 门控)
  std::vector<fleet_msgs::msg::TaskInfo> assign_tasks_batch(
    const std::vector<fleet_msgs::msg::RobotStatus> & robots,
    const std::map<std::string, geometry_msgs::msg::Pose> & waypoint_poses);

  // ── 状态转换 ──

  void mark_task_navigating(const std::string & task_id);
  void mark_task_executing(const std::string & task_id);
  void mark_task_waiting(const std::string & task_id);

  /// 任务回 pending 状态(清除非 fixed 任务的底盘绑定)
  void mark_task_pending(const std::string & task_id);

  /// 任务回 pending 但保留底盘绑定(用于超时/重试耗尽后的保留性回队)
  void mark_task_pending_preserve(const std::string & task_id);

  /// 终结操作(带终端状态保护: 已完成/已失败/已取消的任务不会被覆盖)
  void complete_task(const std::string & task_id);
  void fail_task(const std::string & task_id, const std::string & reason);
  void cancel_task(const std::string & task_id);

  // ── 重试周期控制 ──

  /// 检查任务的保留性回队次数是否已超过限制
  bool would_exceed_retry_cycles(const std::string & task_id, int max_cycles) const;
  int  retry_cycle_count(const std::string & task_id) const;

  // ── 查询 ──

  uint8_t get_task_type(const std::string & task_id) const;
  fleet_msgs::msg::TaskInfo get_task_info(const std::string & task_id) const;
  std::vector<fleet_msgs::msg::TaskInfo> get_all_tasks() const;

  size_t pending_count() const;

  // ── 维护 ──

  /// 从 all_ 重建优先队列(O(n log n))，确保 pending 任务不丢失
  void repair_queue();

  /// 清理已完成的旧任务，最多保留 max_keep 个
  void purge_finished(size_t max_keep = 200);

private:
  void remove_from_queue(const std::string & task_id);
  std::string generate_id();

  rclcpp::Node * node_;

  std::priority_queue<fleet_msgs::msg::TaskInfo,
                      std::vector<fleet_msgs::msg::TaskInfo>,
                      TaskCompare> queue_;          // 待分配任务的优先队列

  std::map<std::string, fleet_msgs::msg::TaskInfo> all_;  // 全量任务索引(task_id → TaskInfo)
  std::set<std::string> fixed_;                            // 固定分配任务集(指定了 robot_id)
  std::map<std::string, geometry_msgs::msg::Pose> waypoint_poses_;

  int counter_{0};                                        // 任务 ID 自增计数器
  std::map<std::string, int> fcfc_skip_count_;            // FCFS 门控连续阻塞计数
  std::map<std::string, int> retry_cycle_count_;          // 保留性回队次数
  static constexpr int kFcfcGateMaxSkips = 30;            // FCFS 门控超时(~30s)

  mutable std::recursive_mutex mutex_;
};

}  // namespace fleet_manager

#endif  // FLEET_MANAGER__TASK_SCHEDULER_HPP_
