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

// ============================================================================
// 类型别名 — 简化 Nav2 action 接口引用
// ============================================================================

using NavigateToPose       = nav2_msgs::action::NavigateToPose;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using FollowWaypoints      = nav2_msgs::action::FollowWaypoints;
using GoalHandleNavigate   = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using GoalHandleNavigateThrough = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

// ============================================================================
// 机器人导航状态 — 每台底盘维护一份，记录当前导航任务的所有上下文
// ============================================================================

struct RobotNavInfo
{
  // ── Nav2 导航 ──
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client;  // Nav2 action 客户端
  GoalHandleNavigate::SharedPtr goal_handle;                    // 当前活跃 goal 的句柄
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr align_client;
  GoalHandleNavigateThrough::SharedPtr align_goal_handle;
  bool aligning_before_nav{false};
  bool route_alignment_done{false};
  bool has_active_goal{false};                                  // 是否有正在执行的导航指令
  uint64_t nav_seq{0};                                          // 导航序列号(取消旧 goal 时递增)
  rclcpp::Time nav_since;                                       // 当前 goal 开始时间(卡死检测用)
  rclcpp::Time nav_last_activity;                               // 最近一次导航活动时间
  rclcpp::Time recent_cancel_until;                             // 取消后的静默期(防止重复发送)

  // ── 逐航点路径 ──
  std::string current_task_id;          // 当前执行的任务 ID
  std::vector<std::string> route;       // 规划的航点路径(有序)
  size_t route_index{0};                // 当前所在的路径索引
  int retry_count{0};                   // 当前跳的重试次数
  rclcpp::Time retry_after;            // 下次重试的允许时间(退避窗口)

  // ── 底盘任务(LOAD/UNLOAD/SITE_SPECIFIC) ──
  rclcpp::Publisher<fleet_msgs::msg::TaskCmd>::SharedPtr  task_cmd_pub;
  rclcpp::Subscription<fleet_msgs::msg::TaskFb>::SharedPtr task_fb_sub;
  bool   chassis_task_sent{false};      // 已向底盘发送任务指令
  bool   chassis_handshake_ok{false};   // 底盘握手成功
  bool   chassis_acked{false};          // 底盘已确认
  rclcpp::Time chassis_hs_deadline;     // 握手超时时刻
  rclcpp::Time chassis_exec_deadline;   // 执行超时时刻
  int    chassis_retries{0};            // 底盘握手重试次数
  uint8_t  pending_task_type{1};
  uint32_t pending_site_code{0};
  uint16_t pending_wp_num{0};
  uint64_t pending_task_num{0};

  // ── LED 状态灯 ──
  rclcpp::Publisher<fleet_msgs::msg::LEDTask>::SharedPtr  led_pub;
  rclcpp::Subscription<fleet_msgs::msg::LEDStatus>::SharedPtr led_sub;
  uint8_t last_led_state{0xFF};         // 上次下发的 LED 状态
  uint8_t chassis_led{0xFF};            // 底盘回报的 LED 状态
  bool    led_received{false};          // 是否收到过 LED 状态回报
};

// ============================================================================
// 链式撤退 — 多车互锁时的有序协调
// ============================================================================

/// 撤退链中的一个步骤：指定底盘移动到目标航点(单跳)
struct RetreatChainStep
{
  std::string robot_id;     // 执行此步骤的底盘
  std::string target_wp;    // 目标航点(仅单跳)
};

/// 撤退链执行计划 — 维护链式协调的完整状态机
struct ChainRetreatPlan
{
  std::string original_requester;       // 原始请求者(被阻塞的底盘)
  std::string original_target;          // 请求者的原始目标航点
  std::string original_task_id;         // 请求者的原始任务 ID

  // 参与链协调的底盘的任务快照(链完成后恢复)
  std::map<std::string, std::string> saved_task_ids;   // robot_id → 任务ID
  std::map<std::string, std::string> saved_targets;    // robot_id → 目标航点

  std::vector<RetreatChainStep> steps;  // 有序的撤退步骤
  size_t current_step{0};               // 当前执行到的步骤索引
  bool active{false};                   // 链是否正在执行
  rclcpp::Time started_at;             // 链启动时间(超时检测)
  int step_retry_count{0};             // 当前步骤的重试次数
};

enum class TaskWaitState
{
  NONE,
  WAIT_BLOCKER_RELEASE,
  WAIT_TARGET_CLEAR,
  WAIT_ROUTE_CLEAR,
  SELF_RELOCATING
};

struct TaskWaitCondition
{
  TaskWaitState state{TaskWaitState::NONE};
  std::string robot_id;
  std::string task_id;
  std::string blocker_id;
  std::string from_wp;
  std::string to_wp;
  std::string target_wp;
  int retreat_count{0};
};

// ============================================================================
// FleetManagerNode — 车队调度中枢节点
// ============================================================================

class FleetManagerNode : public rclcpp::Node
{
public:
  FleetManagerNode();
  ~FleetManagerNode() override;

private:
  // ========================================================================
  // 定时器 (单线程执行器，回调之间不会并发，递归锁是防御性措施)
  // ========================================================================

  void control_timer_callback();    // 主循环 500ms: 位置更新→到达检测→调度→死锁检测
  void fast_timer_callback();       // 快循环 200ms: LED 状态 + 导航重试

  // ========================================================================
  // 状态订阅
  // ========================================================================

  /// 接收 fleet_monitor 发布的车队状态，更新在线/离线状态及位置
  void fleet_status_callback(const fleet_msgs::msg::FleetStatus::SharedPtr msg);

  // ========================================================================
  // ROS2 服务
  // ========================================================================

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

  /// 发布带交通管制信息的车队状态(/fleet_manager/fleet_status_traffic)
  void publish_traffic_fleet_status();

  /// 发布运营指标(/fleet_manager/metrics, 每5s)
  void publish_metrics();

  // ========================================================================
  // 任务调度
  // ========================================================================

  /// 调度主循环: 过期清理→孤儿恢复→分配待处理任务
  void schedule_tick();

  /// 将 pending 任务分配给可用底盘(含瓶颈冲突检测和 waiting_fleet 重调度)
  void assign_pending_tasks();

  /// 构建阻塞图并检测死锁环，打破时物理移走 victim
  void deadlock_check();

  // ========================================================================
  // 导航控制
  // ========================================================================

  /// 启动导航任务: 寻路→验证 zone 所有权→预留首跳→启动逐航点导航
  bool start_navigation(const std::string & robot_id,
                        const std::string & target_wp,
                        const std::string & task_id);

  /// 向路径上的下一个航点发送 Nav2 goal(自动跳过已到达的航点)
  void navigate_to_next_waypoint(const std::string & robot_id);

  /// 发送到任意航点的 NavigateToPose action goal
  void navigate_to_waypoint(const std::string & robot_id,
                            const std::string & wp_id,
                            const std::string & task_id,
                            bool is_final,
                            bool alignment_done = false);

  /// Nav2 goal 成功到达的回调: CRUISE 任务完成; LOAD/UNLOAD 任务转底盘执行
  void on_nav_succeeded(const std::string & robot_id,
                        const std::string & task_id);

  /// 导航超时检测: 卡住(stuck 20s)→重试; 绝对超时(45s)→回队列
  void check_arrivals();

  // ========================================================================
  // 链式撤退 — 瓶颈互锁时的有序协调
  // ========================================================================

  /// 底盘是否完全空闲(无活跃 goal、无任务绑定、无底盘执行、路径为空)
  bool is_robot_idle(const std::string & robot_id) const;

  /// 底盘是否静止(不在移动也不在底盘操作中)——可参与链式协调
  bool is_robot_stationary(const std::string & robot_id) const;

  /// 检测互锁: blocker 的所有出口都必须经过 requester 的位置
  bool is_mutual_block(const std::string & blocker, const std::string & blocker_wp,
                       const std::string & requester, const std::string & requester_wp) const;

  /// 递归推占据者: 把 wp 上的机器人推到分支空闲航点(最多4层)
  bool try_push_occupant(const std::string & wp,
                         const std::set<std::string> & excluded,
                         std::set<std::string> & visited,
                         int depth,
                         std::vector<RetreatChainStep> & steps);

  /// 递归构建撤退链(深度限制5层)，成功时链步骤写入 chain_plan_
  bool try_build_retreat_chain(const std::string & requester, const std::string & from_wp,
                               const std::string & to_wp, const std::string & blocker,
                               const std::set<std::string> & blocked_set, int depth);

  /// 执行链中的当前步骤(发送 NavigateToPose)
  void execute_chain_step();

  /// 链步骤完成的回调: 成功→推进下一步; 失败→重试→超限则 abort
  void on_chain_step_complete(const std::string & robot_id, bool nav_success);

  /// 中止链并恢复所有参与底盘的任务(保留绑定重新入队)
  void abort_chain(const std::string & reason);

  // ========================================================================
  // 底盘任务控制 (LOAD/UNLOAD/SITE_SPECIFIC)
  // ========================================================================

  void send_chassis_cmd(const std::string & robot_id,
                        const std::string & task_id,
                        uint16_t wp_num, uint8_t type, uint32_t site);

  void send_chassis_ack(const std::string & robot_id, uint64_t task_num);

  void chassis_fb_callback(const std::string & robot_id,
                           const fleet_msgs::msg::TaskFb::SharedPtr msg);

  void chassis_timeout_check();

  bool is_robot_executing(const std::string & robot_id) const;

  /// 任务完成后的清理: 释放预约→清空导航状态→自动驶离死胡同
  void finalize_task_completion(const std::string & robot_id,
                                const std::string & task_id);

  /// 航点 ID 转为 uint16(用于底盘通信)
  uint16_t wp_to_u16(const std::string & wp_id) const;

  // ========================================================================
  // LED 状态灯
  // ========================================================================

  /// 每 200ms 推送 LED 状态(行走/等待/执行/空闲)
  void led_timer_callback();

  uint8_t determine_led_state(const std::string & robot_id) const;

  void led_status_callback(const std::string & robot_id,
                           const fleet_msgs::msg::LEDStatus::SharedPtr msg);

  // ========================================================================
  // 工具函数
  // ========================================================================

  /// 事件驱动唤醒: 当 blocker 完成任务时，唤醒所有等待它的请求者
  void wake_waiters(const std::string & blocker_id);

  std::vector<std::string> plan_route_for_task(const std::string & robot_id,
      const std::string & target_wp) const;

  std::string find_active_route_conflict(const std::string & robot_id,
      const std::string & task_id,
      const std::vector<std::string> & path,
      std::string & conflict_task_id,
      std::string & conflict_resource) const;

  std::string physical_waypoint_blocker(const std::string & robot_id,
      const std::string & wp_id) const;

  /// 图感知搜索: 找最近空闲且路径上无占用的航点 (统一函数)
  std::string find_safe_free_waypoint(const std::string & from_wp,
      const std::set<std::string> & exclude,
      const std::string & self_robot = "") const;

  void cancel_goals(const std::shared_ptr<RobotNavInfo> & ni);
  void cancel_all_goals();
  void stop_robot(const std::string & robot_id, int burst = 10);
  void stop_all();

  /// 获取或创建指定底盘的导航状态(懒初始化: 创建 action client / topic pub-sub)
  std::shared_ptr<RobotNavInfo> get_or_create_nav(const std::string & robot_id);

  double normalize_angle(double a) const;
  double get_yaw(const geometry_msgs::msg::Quaternion & q) const;
  std::string join_route(const std::vector<std::string> & wps) const;

  // ========================================================================
  // ROS2 I/O
  // ========================================================================

  rclcpp::Subscription<fleet_msgs::msg::FleetStatus>::SharedPtr fleet_sub_;
  rclcpp::Publisher<fleet_msgs::msg::FleetStatus>::SharedPtr   traffic_pub_;
  rclcpp::Publisher<fleet_msgs::msg::TaskInfo>::SharedPtr       task_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           alert_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           metrics_pub_;

  rclcpp::Service<fleet_msgs::srv::SubmitTask>::SharedPtr     submit_srv_;
  rclcpp::Service<fleet_msgs::srv::CancelTask>::SharedPtr     cancel_srv_;
  rclcpp::Service<fleet_msgs::srv::GetRobotStatus>::SharedPtr status_srv_;
  rclcpp::Service<fleet_msgs::srv::LoadTrafficMap>::SharedPtr load_srv_;
  rclcpp::Service<fleet_msgs::srv::SaveTrafficMap>::SharedPtr save_srv_;
  rclcpp::Service<fleet_msgs::srv::RemoveRobot>::SharedPtr    remove_srv_;

  rclcpp::TimerBase::SharedPtr control_timer_;   // 500ms
  rclcpp::TimerBase::SharedPtr fast_timer_;      // 200ms

  /// 各底盘的急停速度指令发布器(按需创建)
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> vel_pubs_;

  // ========================================================================
  // 核心子模块
  // ========================================================================

  std::unique_ptr<TaskScheduler>    scheduler_;   // 任务队列 + 贪心分配
  std::unique_ptr<TrafficManager>   traffic_;     // 交通图管理 + BFS 寻路
  std::unique_ptr<OccupancyManager> occupancy_;   // 区域占用 zone_lock + 预留

  // ========================================================================
  // 全局状态
  // ========================================================================

  mutable std::recursive_mutex mtx_;                          // 全局递归锁
  std::map<std::string, fleet_msgs::msg::RobotStatus> robots_; // 所有已知底盘
  std::map<std::string, std::shared_ptr<RobotNavInfo>> navs_;  // 导航上下文
  std::set<std::string> removed_;                               // 已移除的底盘黑名单

  // 事件驱动等待: blocker → 等待它的请求者列表
  std::map<std::string, std::set<std::string>> waiting_for_;
  std::map<std::string, TaskWaitCondition> task_waits_;

  fleet_msgs::msg::FleetStatus last_fleet_;      // 最新车队状态快照
  bool has_fleet_{false};                        // 是否已收到过车队状态
  rclcpp::Time last_fleet_time_;                 // 上次收到车队状态的时间
  uint64_t tick_{0};                             // 主循环计数

  ChainRetreatPlan chain_plan_;                  // 链式撤退状态机(全局单例)

  std::string prev_cycle_key_;                   // 上次检测到的死锁环特征 key
  rclcpp::Time cycle_first_seen_;                // 当前死锁环首次检测时间
  uint64_t deadlock_break_count_{0};            // 累计死锁打破次数(metrics 用)
  rclcpp::Time last_metrics_time_;               // 上次发布 metrics 的时间

  // ========================================================================
  // 可配置参数 (通过 ROS2 parameter 或 launch 文件设置)
  // ========================================================================

  double waypoint_radius_{0.5};          // 航点到达判定半径(m)
  double segment_lateral_{1.2};          // 线段横向判定距离(m)
  double sched_interval_{1.0};           // 调度周期(s)
  double retry_base_{1.0};               // 退避基础时间(s)
  int    retry_max_{5};                  // 单跳最大重试次数
  double nav_stuck_timeout_{20.0};       // 导航卡住超时(s)
  double nav_absolute_timeout_{45.0};    // 导航绝对超时(s)
  double chassis_hs_timeout_{5.0};       // 底盘握手超时(s)
  double chassis_exec_timeout_{30.0};    // 底盘执行超时(s)
  int    chassis_max_retries_{3};        // 底盘握手最大重试次数
  double monitor_stale_timeout_{4.0};    // fleet_monitor 数据陈旧超时(s)
  double ghost_lock_ttl_{120.0};         // 幽灵锁 TTL(s)
  double deadlock_timeout_{10.0};         // 死锁持续多久后打破(s)
  int    max_task_retry_cycles_{5};      // 任务最大重试轮数
};

}  // namespace fleet_manager

#endif  // FLEET_MANAGER__FLEET_MANAGER_NODE_HPP_
