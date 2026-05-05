# 系统架构

## 总体架构

Fleet Management System 采用分层架构，从上到下依次为：Web 可视化层、REST API 层、ROS2 调度核心层、以及机器人执行层。

```
+----------------------------------------------------------------------+
|                        Web 可视化层 (fleet_web_ui)                     |
|  +----------------------------------+  +---------------------------+  |
|  |  前端 (HTML/CSS/JS)               |  |  FastAPI 后端              |  |
|  |  - 地图实时显示                    |  |  - REST API (航点/任务)    |  |
|  |  - 机器人位置/状态                |  |  - WebSocket 实时推送      |  |
|  |  - 任务管理面板                    |  |  - ROS2 Service Client    |  |
|  |  - 航点编辑                       |  |  - FleetStatus 订阅       |  |
|  +----------------------------------+  +---------------------------+  |
+----------------------------------------------------------------------+
                    | REST API (HTTP) + WebSocket (WS)
                    v
+----------------------------------------------------------------------+
|                    ROS2 调度核心层                                      |
|                                                                        |
|  +----------------------------------+                                  |
|  | Fleet Monitor (fleet_monitor)     |  ---> /fleet_monitor/fleet_status|
|  |  - TF 自动发现机器人命名空间       |                                  |
|  |  - 在线状态判定 (online_flag 心跳) |                                  |
|  |  - 位姿更新 (map->base_footprint) |                                  |
|  |  - 电池监控 (BatteryState)        |                                  |
|  +----------------------------------+                                  |
|          | FleetStatus (订阅)                                           |
|          v                                                             |
|  +------------------------------------------------------------------+ |
|  | Fleet Manager (fleet_manager)      === 核心调度节点 ===             | |
|  |                                                                    | |
|  |  +-------------------+  +-------------------+  +----------------+  | |
|  |  | TaskScheduler     |  | OccupancyManager  |  | TrafficManager |  | |
|  |  | - 优先级队列      |  | - zone_locks      |  | - 地图加载/保存 |  | |
|  |  | - 贪心分配        |  | - reservations    |  | - BFS 寻路     |  | |
|  |  | - FCFS 门控       |  | - ghost_locks     |  | - Dijkstra 加权|  | |
|  |  | - 瓶颈冲突检测    |  | - 幽灵锁 TTL 过期  |  | - 航点查询     |  | |
|  |  | - 死锁检测+打破   |  | - 死胡同自动驶离   |  |               |  | |
|  |  | - 链式撤退        |  |                   |  |               |  | |
|  |  +-------------------+  +-------------------+  +----------------+  | |
|  +------------------------------------------------------------------+ |
|          | Nav2 Action / TaskCmd Topic                                 |
|          v                                                             |
|  +------------------------------------------------------------------+ |
|  | 机器人执行层                                                       | |
|  |  Robot 1 (Nav2 + 底盘 + STM32)                                     | |
|  |  Robot 2 (Nav2 + 底盘 + STM32)                                     | |
|  |  ...                                                               | |
|  +------------------------------------------------------------------+ |
+----------------------------------------------------------------------+

                    +---------------------------+
                    | Zenoh Bridge               |
                    | (跨网络路由, 远程机器人接入) |
                    +---------------------------+
```

---

## 包级职责表

| 包名 | 语言 | 职责 |
|------|------|------|
| **fleet_msgs** | IDL | ROS2 消息（8 种 msg）、服务（6 种 srv）定义，通信协议规范 |
| **fleet_monitor** | C++ | 车队监控：通过 /tf 和话题扫描自动发现机器人，维护在线/离线判定，聚合 FleetStatus 发布 |
| **fleet_manager** | C++ | 调度管理核心：任务调度、交通管制、死锁检测、链式撤退、导航控制、底盘任务协调 |
| **robot_detector** | Python | 通过 Zenoh 扫描 LAN 发现远程机器人，作为 fleet_monitor 的补充发现机制 |
| **traffic_editor** | Python/PyQt5 | 交通图图形化编辑器，支持航点增删改、连接管理、与底图关联、YAML 导入导出 |
| **fleet_management_system** | Python/XML | 系统启动文件、地图数据、诊断/压力测试/仿真脚本、Zenoh 配置 |
| **fleet_web_ui** | Python (FastAPI) | Web 后端：REST API + WebSocket 实时推送，ROS2 桥接，地图可视化，任务管理 |

---

## 核心数据流

```
                    fleet_monitor
                         |
                         | /fleet_monitor/fleet_status (FleetStatus)
                         | 500ms 周期性发布
                         v
                    fleet_manager
                         |
                         |--- TaskScheduler: 任务队列管理
                         |--- OccupancyManager: Zone-based 占用管理
                         |--- TrafficManager: 路径规划
                         |
                         | /fleet_manager/fleet_status_traffic (FleetStatus 含交通信息)
                         | 500ms 周期性发布
                         v
                    fleet_web_ui (server_ros2.py)
                         |
                         | WebSocket 实时推送
                         v
                    Web 前端

     === 下行控制通道 ===

     fleet_manager
         |
         |--- Nav2 Action: /<robot_id>/navigate_to_pose
         |      逐航点导航 goal
         |
         |--- Topic: /<robot_id>/task/assign (TaskCmd)
         |      底盘任务指令 (LOAD/UNLOAD/SITE_SPECIFIC)
         |
         |--- Topic: /<robot_id>/led/task (LEDTask)
         |      状态灯控制
         |
         v
     Robot (底盘)

     === 上行状态通道 ===

     Robot (底盘)
         |
         |--- Topic: /tf (TFMessage)
         |      位姿变换
         |
         |--- Topic: /<robot_id>/battery_state (BatteryState)
         |      电池信息
         |
         |--- Topic: /<robot_id>/online_flag (Int32)
         |      在线心跳
         |
         |--- Topic: /<robot_id>/task/feedback (TaskFb)
         |      底盘任务执行反馈
         |
         |--- Topic: /<robot_id>/led/status (LEDStatus)
         |      LED 状态回报
         |
         v
     fleet_monitor / fleet_manager
```

### 关键 Topic 汇总

| Topic | 类型 | 发布者 | 频率 | 说明 |
|-------|------|--------|------|------|
| `/fleet_monitor/fleet_status` | FleetStatus | fleet_monitor | 2Hz | 聚合车队原始状态 |
| `/fleet_manager/fleet_status_traffic` | FleetStatus | fleet_manager | 2Hz | 含交通管制信息的车队状态 |
| `/fleet_manager/task_status` | TaskInfo | fleet_manager | 事件 | 任务状态变更通知 |
| `/fleet_manager/alerts` | String | fleet_manager | 事件 | 告警消息 |
| `/fleet_manager/metrics` | String | fleet_manager | 0.2Hz | 运营指标快照 |
| `/<robot_id>/task/assign` | TaskCmd | fleet_manager | 事件 | 下发任务指令 |
| `/<robot_id>/task/feedback` | TaskFb | 底盘 | 事件 | 任务执行反馈 |
| `/<robot_id>/led/task` | LEDTask | fleet_manager | 5Hz | LED 状态控制 |
| `/<robot_id>/led/status` | LEDStatus | 底盘 | 5Hz | LED 状态回报 |
| `/<robot_id>/online_flag` | Int32 | 底盘 | 1Hz | 在线心跳 |
| `/<robot_id>/battery_state` | BatteryState | 底盘 | 1Hz | 电池状态 |

### 关键 Service 汇总

| Service | 服务端 | 说明 |
|---------|--------|------|
| `~/submit_task` | fleet_manager | 提交新任务 |
| `~/cancel_task` | fleet_manager | 取消任务 |
| `~/get_robot_status` | fleet_manager / fleet_monitor | 查询机器人状态 |
| `~/load_traffic_map` | fleet_manager | 加载交通图 |
| `~/save_traffic_map` | fleet_manager | 保存交通图 |
| `~/remove_robot` | fleet_manager | 移除机器人出队 |

### 关键 Action 汇总

| Action | 服务端 | 说明 |
|--------|--------|------|
| `/<robot_id>/navigate_to_pose` | Nav2 | 向指定航点导航 |

---

## 调度算法详解

### 任务状态机

任务在其生命周期中经历以下状态转换：

```
                          ┌─────────┐
                          │ pending  │  ← 初始状态，等待分配
                          └────┬─────┘
                               │ assign_tasks_batch() 分配成功
                               v
                          ┌─────────┐
                     ┌───│ assigned │  ← 已分配底盘，准备导航
                     │   └────┬─────┘
                     │        │ start_navigation() 启动导航
                     │        v
                     │   ┌────────────┐
                     │   │ in_progress│  ← 导航中（逐航点前进）
                     │   └─────┬──────┘
                     │         │ on_nav_succeeded() 到达目标航点
                     │         v
                     │   ┌───────────┐
                     │   │ executing │  ← 底盘执行任务 (LOAD/UNLOAD)
                     │   └─────┬─────┘
                     │         │ chassis TaskFb 完成
                     │         v
                     │   ┌───────────┐
                     │   │ completed │  ← 终态：成功完成
                     │   └───────────┘
                     │
                     │   ┌───────────┐
                     ├──→│  failed   │  ← 终态：失败（超时/错误/离线）
                     │   └───────────┘
                     │
                     │   ┌───────────┐
                     └──→│ cancelled │  ← 终态：被取消
                         └───────────┘

           ┌───────────────┐
   ┌──────→│ waiting_fleet │  ← 中间状态：退避重试中
   │       └───────┬───────┘
   │               │ 退避时间到 → 重新入队
   └───────────────┘
```

**状态转换守护规则：**
- 终态（completed / failed / cancelled）不可覆盖
- `mark_task_pending()` 清除非 fixed 任务的底盘绑定
- `mark_task_pending_preserve()` 保留底盘绑定（用于超时恢复）
- `would_exceed_retry_cycles()` 控制最大重试轮数（默认 5 次）

### 分配策略

分配算法位于 `TaskScheduler::assign_tasks_batch()`，采用三层筛选：

**第一层：优先级队列排序**

优先级队列使用最大堆，排序规则：
- `priority` 数值大者优先
- 同优先级按创建时间排序（先创建者优先）
- 同创建时间按 task_id 字典序

**第二层：固定分配（fixed）预留**

已指定 robot_id 的 fixed 任务优先预留指定底盘，预留的底盘从可用池中移除。

**第三层：最近距离贪心 + FCFS 门控**

对每个 pending 任务，遍历可用底盘列表，选择距离目标航点最近的底盘：
- 计算欧氏距离 `hypot(dx, dy)`
- 选择最近者分配
- 分配后从可用池中移除该底盘

**FCFS 门控机制：** 若队首任务因无可用底盘而分配失败，则阻塞后续任务分配。连续阻塞超过 30 个调度 tick（约 30 秒）时，跳过该任务让后续任务先分配，防止队首阻塞导致饥饿。

### 瓶颈冲突检测

在 `FleetManagerNode::assign_pending_tasks()` 中，当一批任务被分配后，执行对向路径冲突检测：

1. 为每个分配的任务计算路径（`TrafficManager::find_path`）
2. 提取每条路径的边集合（`{from, to}` 有序对）
3. 对每一对任务 `(i, j)`，检查是否存在互逆边：
   - `edges_i` 包含 `(A, B)` 且 `edges_j` 包含 `(B, A)`
4. 若存在对向冲突，defer 低优先级任务：
   - 调用 `mark_task_pending_preserve()` 保留底盘绑定
   - 设置 `retry_after = now + 5s` 退避延迟
   - 发布更新后的 TaskInfo

### 死锁检测

位于 `FleetManagerNode::deadlock_check()`，在主循环中每次 `control_timer_callback` 执行：

**阻塞图构建：**
- 遍历所有活跃导航的底盘
- 对每个底盘，查询其路径上下一跳航点的阻塞者
- 阻塞关系：`robot_A → robot_B` 表示 A 在等待 B 占用的航点

**DFS 环检测：**

```
for each robot R in block_graph:
    chain = []
    visited = {}
    cur = R
    while cur is not None:
        if cur in visited:
            cycle = chain[visited[cur]:]  // 发现环
            break
        visited[cur] = chain.length
        chain.append(cur)
        cur = block_graph[cur]            // 跟随阻塞链
    if cycle found: break
```

**环特征去重：** 将环中 robot_id 排序后拼接为 key（旋转无关），连续两次同环才触发打破（`deadlock_timeout_ = 10s`）。

**Victim 选择与打破：**
1. 从环中选择最低优先级任务的底盘作为 victim
2. 取消 victim 的导航 goal，释放预留
3. 调用 `occupancy_->find_nearest_free_waypoint()` 找到最近空闲航点
4. 给 victim 发送 relocate 导航指令物理移走
5. 原任务调用 `mark_task_pending_preserve()` 保留绑定回队

### 链式撤退

当两车（或多车）形成互锁且死锁检测尚未触发时，链式撤退提供更优雅的协调机制。

**触发条件：** 在 `navigate_to_next_waypoint()` 中，当导航跳被阻塞 ≥ 2 次时：
- 检查阻塞者是否静止（`is_robot_stationary`）
- 检查是否互锁（`is_mutual_block`：阻塞者的所有出口都被请求者堵塞）

**递归链构建** (`try_build_retreat_chain`, 最大深度 5)：

```
build_chain(requester, from_wp, to_wp, blocker, blocked_set, depth):
    if depth >= 5: return false
    
    for each neighbor of from_wp (except to_wp and blocked_set):
        if neighbor is free:
            best_retreat = neighbor  // 直接空闲最优
            break
        if neighbor occupied by stationary robot:
            // 递归：让占用者撤退
            subchain = build_chain(occupier, neighbor, from_wp, ...)
            track shortest subchain
    
    if no retreat found: return false
    
    chain = [subchain..., (requester → retreat), (blocker → exit), (requester → to_wp)]
    return true
```

**链执行** (`execute_chain_step`)：
- 每步向目标航点发送 NavigateToPose goal
- 步骤完成 → 推进 `current_step++`
- 单步失败 → 最多重试 2 次 → 超限则 `abort_chain`
- 整体超时 180s → 自动 `abort_chain`
- 链完成后 → 恢复所有参与底盘的原始任务

---

## 占用管理

### Zone 模型

OccupancyManager 采用基于 Zone 的区域占用管理模型：

```
Zone(W) = {航点W} ∪ {所有与W相邻的边}

    (A)────(B)         ← 机器人 R1 占据 Zone(B) = {B, (A-B), (B-C)}
           │
          (C)

若 R2 想从 A 走到 C，必须等待 R1 离开 B（或其 Zone 覆盖的边）
```

### 三层保护机制

| 层级 | 结构 | 生命周期 | 说明 |
|------|------|----------|------|
| **zone_locks_** | `map<waypoint_id, robot_id>` | 持续持有，update_location 刷新 | 物理占用：底盘当前位置的 zone |
| **reservations_** | `map<robot_id, waypoint_id>` | 跨越下一航点时创建 | 预留：即将前往的下一航点，TTL=300s |
| **ghost_locks_** | `map<robot_id, timestamp>` | 离线后 TTL=120s | 离线底盘的 zone 保留（幽灵守卫）|

### 位置更新流程

`OccupancyManager::update_location()` 将连续位姿映射到离散元素：

```
输入: robot_id, pose (x, y)

第一步: 判断是否在航点范围内
  for each waypoint wp:
    dist = hypot(pose - wp.pose)
    cap = max(capture_radius, wp.radius)
    if dist <= cap: → type=WAYPOINT, waypoint_id=wp

第二步: 若不在任何航点，判断是否在航段上
  for each edge (A, B):
    dist = point_to_segment_distance(pose, A, B)
    if dist <= segment_lateral_max: → type=SEGMENT, segment_from=A, segment_to=B

第三步: 释放旧 zone_locks，设置新 zone_locks
  碰撞保护: 绝不覆盖其他底盘的锁（falla collision warning）
```

### 安全检查

`can_enter(robot_id, from_wp, to_wp)`:
1. 检查 `to_wp` 的 zone_lock 是否被其他底盘持有
2. 检查 `to_wp` 是否被其他底盘预留
3. 全部通过 → 返回空串（可通行）

`waypoint_blocker(robot_id, wp_id)`:
- 返回阻塞该航点的底盘 ID（zone_lock 持有者或预留者）
- 用于构建死锁检测的阻塞图

`reserve_next(robot_id, from_wp, to_wp)`:
- 先做 `can_enter` 检查
- 通过后记录预留，5 分钟内未使用则过期

---

## 通信协议

### ROS2 消息定义 (fleet_msgs)

| 消息 | 字段 | 说明 |
|------|------|------|
| **FleetStatus** | timestamp, robots[], pending_tasks[], active_tasks, system_status | 车队聚合状态 |
| **RobotStatus** | robot_id, namespace, connection_status, battery, current_pose, current_task_id, nav_status, location_type, current_waypoint, current_segment, planned_route[], last_update | 单机状态 |
| **TaskInfo** | task_id, waypoint_id, target_pose, task_type, site_code, status, assigned_robot_id, priority, created_at, started_at, completed_at | 任务信息 |
| **TaskCmd** | task_id, waypoint_id, task_type, site_code, ack | 底盘任务指令（也用于 ACK） |
| **TaskFb** | task_id, status(0=握手/1=完成/2=错误), error_code | 底盘任务反馈 |
| **Waypoint** | waypoint_id, name, pose, connections[], is_parking_spot, is_charging_station, radius | 航点定义 |
| **TrafficMap** | map_id, map_name, map_yaml_path, waypoints[], created_at, modified_at | 交通图 |
| **LEDTask** | state (0=行走/1=交通等待/2=执行中/3=空闲) | LED 状态控制 |
| **LEDStatus** | state, received | LED 状态回报 |

### 服务请求/响应

| 服务 | 请求 | 响应 |
|------|------|------|
| **SubmitTask** | waypoint_id, priority, robot_id(可选), task_type, site_code | success, task_id, message |
| **CancelTask** | task_id | success, message |
| **GetRobotStatus** | robot_id(可为空) | success, robots[], message |
| **LoadTrafficMap** | file_path | success, map, message |
| **SaveTrafficMap** | file_path | success, message |
| **RemoveRobot** | robot_id | success, message |

### 底盘任务通信协议

TaskCmd/TaskFb 构成完整的握手-执行-确认流程：

```
fleet_manager                    底盘 (OPi/STM32)
     │                              │
     │── TaskCmd(task_type, ack=0) ─→│  下发任务指令
     │                              │
     │←── TaskFb(status=0) ────────│  握手成功 (HANDSHAKE_OK)
     │                              │
     │── TaskCmd(ack=1) ───────────→│  确认收到握手
     │                              │
     │        [底盘执行任务]          │
     │                              │
     │←── TaskFb(status=1/2) ──────│  完成/错误 (COMPLETED/ERROR)
     │                              │
     │── TaskCmd(ack=1) ───────────→│  最终确认
     │                              │
```

**错误码定义（TaskFb.error_code）：**
- 0x00 = 未知错误
- 0x01 = 无效参数
- 0x02 = 串口通信故障 (OPi <-> STM32)
- 0x03 = 执行器无响应 (STM32 <-> actuator)
- 0x04 = 动作执行失败（如夹爪滑落）
- 0x05 = 动作执行超时 (30s)

---

## 关键参数表

所有参数通过 ROS2 Parameter 设定，可在 launch 文件中配置或运行时动态修改。

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `traffic_map_file` | "" | 交通图 YAML 文件路径 |
| `waypoint_acceptance_radius` | 0.5 | 航点到达判定半径 (m) |
| `traffic_segment_lateral_max` | 1.2 | 航段横向判定距离 (m) |
| `scheduler_interval_sec` | 1.0 | 调度周期 (s) |
| `nav_retry_base_sec` | 1.0 | 导航退避基础时间 (s) |
| `nav_retry_max` | 5 | 单跳最大重试次数 |
| `nav_stuck_timeout_sec` | 20.0 | 导航卡住超时 (s) |
| `nav_absolute_timeout_sec` | 45.0 | 导航绝对超时 (s) |
| `chassis_handshake_timeout_sec` | 5.0 | 底盘握手超时 (s) |
| `chassis_exec_timeout_sec` | 30.0 | 底盘执行超时 (s) |
| `chassis_max_retries` | 3 | 底盘握手重试次数 |
| `monitor_fleet_stale_timeout_sec` | 4.0 | fleet_monitor 数据陈旧超时 (s) |
| `ghost_lock_ttl_sec` | 120.0 | 幽灵锁 TTL (s) |
| `deadlock_timeout_sec` | 10.0 | 死锁持续超时 (s) |
| `max_task_retry_cycles` | 5 | 任务最大保留性回队轮数 |
| `persist_log_enabled` | true | 是否启用持久化日志 |
| `persist_log_dir` | "test_logs" | 持久化日志目录 |

**内部常量（代码中硬编码）：**

| 常量名 | 值 | 说明 |
|--------|-----|------|
| `kMaxChainDepth` | 5 | 链式撤退最大深度 |
| `kChainTotalTimeout` | 180s | 链式撤退整体超时 |
| `kChainStepTimeout` | 30s | 链式撤退单步超时 |
| `kMaxChainStepRetries` | 2 | 链步最大重试次数 |
| `kNavCancelSettlingSec` | 1.0s | 导航取消静默期 |
| `kReservationTTL` | 300s | 预留有效期 |
| `kFcfcGateMaxSkips` | 30 | FCFS 门控最大跳过 tick 数 |
| `kOccupiedEdgeCost` | 15.0 | 被占边 Dijkstra 惩罚代价 |
| `kOccupiedWaypointCost` | 10.0 | 被占航点 Dijkstra 惩罚代价 |
