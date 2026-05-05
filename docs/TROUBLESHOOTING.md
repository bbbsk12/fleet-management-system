# 故障排除

## 目录

1. [机器人不上线](#机器人不上线)
2. [任务卡在 pending](#任务卡在-pending)
3. [任务卡在 in_progress](#任务卡在-in_progress)
4. [死胡同堵塞](#死胡同堵塞)
5. [死锁反复触发](#死锁反复触发)
6. [内存CPU 过高](#内存cpu-过高)
7. [Web UI 连接失败](#web-ui-连接失败)
8. [Zenoh 连接问题](#zenoh-连接问题)
9. [底盘通信超时](#底盘通信超时)
10. [日志分析](#日志分析)

---

## 机器人不上线

### 症状

- Web 控制台 Fleet Monitor 页面中某机器人状态持续显示为 `offline`。
- `robots_online` 话题数值小于预期。
- 调度器日志中出现 `[WARN] [chassis] robot_XX handshake timeout`。
- `ros2 topic echo /robot_XX/status` 无数据。

### 原因

1. **Zenoh 桥接未启动或配置错误**：机器人端与系统主控之间的 Zenoh 会话未建立。
2. **`online_flag` 话题未发布**：机器人节点未启动或 `online_flag` 发布器故障。
3. **网络不通**：工控机与机器人底盘之间的物理网络或 WiFi 连接中断。
4. **底盘驱动未启动**：机器人端 ROS2 节点崩溃或无响应。
5. **配置不匹配**：`ROS_DOMAIN_ID` 不一致，导致不同 ROS2 域中节点无法相互发现。

### 解决方案

1. **检查 Zenoh 桥接**：
   ```bash
   # 确认 Zenoh router 是否运行
   ps aux | grep zenoh
   # 重启 Zenoh 桥接
   ros2 run fleet_bridge zenoh_bridge --config config/zenoh_bridge.json
   ```

2. **检查 `online_flag` 话题**：
   ```bash
   # 查看是否有在线标志发布
   ros2 topic list | grep online_flag
   ros2 topic echo /online_flag
   ```
   若无数据，检查机器人端主程序是否正常启动。

3. **检查网络连通性**：
   ```bash
   ping <robot_ip_address>
   # 检查指定端口可达性
   nc -zv <robot_ip_address> 7447
   ```

4. **查看 fleet_monitor 日志**：
   ```bash
   # 查找 handshake 相关日志
   grep "handshake" /var/log/fleet_system/monitor.log
   grep "register" /var/log/fleet_system/monitor.log
   ```

5. **统一 ROS_DOMAIN_ID**：
   ```bash
   # 在所有节点设置相同的 domain ID
   export ROS_DOMAIN_ID=42
   ```

---

## 任务卡在 pending

### 症状

- 任务提交后长时间停留在 `pending` 状态。
- Web 控制台任务队列可见任务但无机器人接手。
- `tasks_pending` 话题数值持续为正且不下降。

### 原因

1. **无可用机器人**：所有机器人状态为 `offline` 或 `charging`，调度器无法分配。
2. **机器人均忙碌**：所有在线机器人已达到最大并发任务数上限。
3. **起始 / 目标 waypoint 无效**：任务指定的 waypoint 在地图中不存在或已被禁用。
4. **调度器异常**：调度主循环停止或 `sched_interval` 设置过大。
5. **锁冲突**：任务路径上的关键 waypoint 被永久锁定（ghost lock 未释放）。

### 解决方案

1. **检查机器人状态**：
   ```bash
   ros2 topic echo /robots_online
   ros2 service call /list_robots fleet_msgs/srv/ListRobots
   ```
   确认至少有 1 台机器人处于 `idle` 状态。

2. **检查 waypoint 有效性**：
   ```bash
   # 确认 waypoint 存在于地图中
   ros2 service call /validate_waypoint fleet_msgs/srv/ValidateWaypoint "{waypoint: 'wp_name'}"
   ```

3. **重启调度器主循环**：
   ```bash
   # 检查调度器是否正在运行
   ros2 node list | grep fleet_manager
   # 若进程挂起，重启节点
   ros2 run fleet_manager fleet_manager
   ```

4. **手动触发调度**：
   ```bash
   ros2 service call /force_schedule fleet_msgs/srv/ForceSchedule
   ```

5. **释放幽灵锁**：
   ```bash
   ros2 service call /purge_ghost_locks fleet_msgs/srv/PurgeGhostLocks
   ```

---

## 任务卡在 in_progress

### 症状

- 任务状态为 `in_progress`，但机器人长时间未移动到目标点。
- 机器人位置话题显示机器人停止不动或在某处徘徊。
- 导航相关日志出现异常。

### 原因

1. **Nav2 Action Server 未响应**：机器人端 Nav2 节点崩溃或卡死。
2. **机器人定位丢失**：AMCL 或 localization 节点定位漂移、粒子发散。
3. **目标 waypoint 不可达**：地图拓扑上目标点被隔离（无 lane 连通）或物理通道被堵塞。
4. **路径规划失败**：起点到终点之间不存在有效路径。
5. **机器人急停未恢复**：机器人处于 `emergency_stop` 状态未执行 `resume`。

### 解决方案

1. **检查 Nav2 状态**：
   ```bash
   # 查看 Nav2 action server 是否在运行
   ros2 action list | grep navigate_to_pose
   # 检查 nav2 生命周期节点状态
   ros2 lifecycle list /nav2_lifecycle_manager
   ```

2. **检查定位质量**：
   ```bash
   # 查看粒子分布
   ros2 topic echo /particle_cloud --once
   # 检查 TF 树
   ros2 run tf2_tools view_frames.py
   ```
   若定位丢失，手动初始化定位或重新触发 AMCL 重定位。

3. **验证路径可达性**：
   ```bash
   ros2 service call /check_path fleet_msgs/srv/CheckPath "{start: 'current_wp', end: 'target_wp'}"
   ```

4. **恢复机器人导航**：
   ```bash
   # 取消当前导航目标
   ros2 action send_goal /navigate_to_pose cancel
   # 重新发送目标
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 2.0}}}}"
   ```

5. **手动结束卡死任务**：
   ```bash
   ros2 service call /emergency_stop fleet_msgs/srv/EmergencyStop "{robot_id: 'robot_XX'}"
   ros2 service call /resume_robot fleet_msgs/srv/ResumeRobot "{robot_id: 'robot_XX'}"
   ```

---

## 死胡同堵塞

### 症状

- 机器人到达某 waypoint 后无法继续前进，但前方并无物理障碍。
- 日志中出现 `[WARN] [deadlock] Robot stuck at wp_XXX, auto-relocate triggered`。
- 机器人在两个相邻 waypoint 之间反复折返。

### 原因

- 地图拓扑中存在死胡同（dead-end），机器人进入后只能原路返回。
- 死胡同 waypoint 的 lane 方向配置错误（应配置为 bidirectional 但实际为单向进入）。
- `auto_relocate` 功能检测到机器人长时间未移动，自动尝试重新定位。

### 解决方案

1. **检查地图拓扑**：在 traffic_editor 中查看死胡同 waypoint 的 lane 连接，确保存在出口 lane。
2. **自动重定位机制**：
   - 系统默认启用 `auto_relocate`，当机器人在同一 waypoint 停留超过阈值时自动触发重新规划。
   - 可在 `settings.yaml` 中配置重定位等待时间：
     ```yaml
     auto_relocate: true
     relocate_timeout: 15  # 秒
     ```
3. **强制重定位**：
   ```bash
   ros2 service call /force_relocate fleet_msgs/srv/ForceRelocate "{robot_id: 'robot_XX'}"
   ```
4. **手动引导出死胡同**：通过 Web 控制台向机器人发送一个目标点为死胡同外部 waypoint 的新任务。
5. **修正地图**：在死胡同 waypoint 处添加一个折返 lane，或将其标记为 `dead_end` 类型使调度器避免调度进入。

---

## 死锁反复触发

### 症状

- `deadlock_breaks` 指标在短时间内频繁增长。
- 日志中持续出现 `[WARN] [deadlock] Deadlock detected, breaking chain at robot_XX`。
- 系统吞吐量下降，任务完成率降低。

### 原因

1. **`deadlock_timeout` 设置过小**：正常等待被误判为死锁。
2. **`chain_depth` 限制不合理**：检测深度过浅，无法识别间接循环依赖。
3. **地图拓扑缺陷**：存在不可解的死锁环（如单向闭合环路）。
4. **机器人密度过高**：通道中的机器人密度超过拓扑承载能力。
5. **优先级分布不合理**：多个高优先级任务互相阻塞，死锁解除后快速重建。

### 解决方案

1. **调整死锁参数**：
   ```yaml
   # settings.yaml
   deadlock_timeout: 10   # 从 5 增大至 10 秒，减少误判
   chain_depth: 10        # 从 5 增大至 10，检测更深的依赖链
   ```
   修改后重启 fleet_manager 或通过 Web 控制台热加载。

2. **优化地图拓扑**：
   - 检查死锁热点的 lane 方向，确保存在旁路路径。
   - 在关键路段添加缓冲区 zone，降低机器人密度。
   - 避免超过 4 条 lane 汇聚于同一个 waypoint。

3. **降低机器人密度**：
   - 减少同时调度的并发任务数。
   - 分区域调度，避免大量机器人集中同一区域。

4. **启用自适应优先级**：
   ```yaml
   adaptive_priority: true   # 死锁链中自动降低阻塞者的优先级
   ```

5. **分析死锁日志**：定位频繁参与死锁的机器人和 waypoint，针对性地调整地图拓扑。

---

## 内存/CPU 过高

### 症状

- 系统 CPU 占用持续超过 80%。
- 内存占用持续增长，系统响应变慢。
- 日志中出现 `[WARN] [system] Memory usage above threshold: 85%`。

### 原因

1. **任务队列堆积**：`tasks_pending` 过多导致调度器每轮循环负载上升。
2. **`purge_finished` 未启用**：已完成任务未清理，内存持续增长。
3. **修复队列（repair_queue）膨胀**：异常任务反复进入修复流程，队列无上限。
4. **ROS2 话题队列堆积**：高频率话题（如 `/tf`、`/scan`）的订阅队列长度设置过大。
5. **WebSocket 连接泄漏**：Web 前端频繁重连导致后端连接句柄未释放。

### 解决方案

1. **启用自动清理**：
   ```yaml
   purge_finished: true
   purge_interval: 300   # 每 300 秒清理一次
   ```

2. **限制修复队列**：
   ```yaml
   repair_queue_limit: 100
   ```

3. **调整话题 QoS 设置**，减小队列深度：
   ```cpp
   // 在订阅高频话题时使用较小的队列
   rclcpp::QoS(10).reliable().durability_volatile();
   ```

4. **检查 WebSocket 连接**：
   ```bash
   # 查看当前 WebSocket 连接数
   ss -tlnp | grep 8000 | wc -l
   # 重启 Web 后端释放泄漏连接
   sudo systemctl restart fleet_web
   ```

5. **监控资源并设置告警阈值**：
   ```yaml
   resource_alerts:
     cpu_threshold: 80    # 百分比
     mem_threshold: 85    # 百分比
   ```

---

## Web UI 连接失败

### 症状

- 浏览器访问 Web 控制台地址时显示 "Connection Failed" 或白屏。
- 页面加载后 Dashboard 数据不刷新，显示 "No data"。
- 浏览器控制台显示 WebSocket 连接错误。

### 原因

1. **CORS 配置错误**：前端请求被后端 CORS 策略拦截。
2. **WebSocket 握手失败**：前端 WebSocket URL 配置与后端不一致。
3. **后端进程未运行**：Flask 应用崩溃或端口被占用。
4. **防火墙阻止**：系统防火墙阻止了前端对后端端口（8000）的访问。
5. **前端构建产物缺失**：`npm build` 未执行或构建失败。

### 解决方案

1. **检查后端进程**：
   ```bash
   # 确认后端运行状态
   ps aux | grep flask
   # 查看后端日志
   journalctl -u fleet_web -n 50 --no-pager
   # 重启后端
   sudo systemctl restart fleet_web
   ```

2. **检查端口占用**：
   ```bash
   ss -tlnp | grep 8000
   # 若端口被占用，停止冲突进程
   sudo fuser -k 8000/tcp
   ```

3. **检查 CORS 配置**：
   编辑 `backend/app.py`，确认 CORS 允许的 origin 包含前端实际域名：
   ```python
   CORS(app, origins=["http://localhost:5173", "http://<actual_domain>"])
   ```

4. **检查 WebSocket URL**：
   确认前端配置中的 WebSocket URL 指向正确的后端地址：
   ```
   ws://<backend_host>:8000/ws
   ```

5. **重新构建前端**：
   ```bash
   cd frontend
   npm run build
   sudo systemctl restart nginx   # 若使用 nginx 代理
   ```

6. **检查防火墙**：
   ```bash
   sudo ufw status
   sudo ufw allow 8000/tcp
   ```

---

## Zenoh 连接问题

### 症状

- 机器人端与主控端无法通信。
- 日志中出现 `[ERROR] [zenoh_bridge] Connection refused` 或 `[ERROR] [zenoh_bridge] Session timeout`。
- `ros2 topic list` 只能看到本机话题，看不到远程话题。

### 原因

1. **Zenoh Router 地址配置错误**：桥接配置中的 router 端点地址不正确。
2. **Zenoh Router 未运行**：主控端 Zenoh router 进程崩溃或未启动。
3. **网络防火墙**：Zenoh 使用的端口（默认 7447）被防火墙阻断。
4. **桥接配置错误**：`config/zenoh_bridge.json` 中的配置参数（如 mode、connect 端点）不正确。
5. **ROS_DOMAIN_ID 不一致**：主控与机器人端的 ROS_DOMAIN_ID 不同。

### 解决方案

1. **确认 Zenoh Router 运行**：
   ```bash
   # 启动 Zenoh router
   zenohd -c config/zenoh_router.json
   # 或使用 Docker
   docker run -d --net host eclipse/zenoh:latest
   ```

2. **验证桥接配置**：
   ```bash
   # 检查配置文件
   cat config/zenoh_bridge.json
   ```
   确保配置包含正确的 router 地址：
   ```json
   {
     "mode": "client",
     "connect": {
       "endpoints": ["tcp/<router_ip>:7447"]
     },
     "transport": {
       "unicast": {
         "accept": ["tcp/0.0.0.0:7447"]
       }
     }
   }
   ```

3. **测试网络连通性**：
   ```bash
   ping <router_ip>
   nc -zv <router_ip> 7447
   ```

4. **统一 ROS_DOMAIN_ID**：
   ```bash
   export ROS_DOMAIN_ID=42
   # 在 .bashrc 中持久化
   echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
   ```

5. **启用 Zenoh 调试日志**：
   ```bash
   RUST_LOG=debug zenohd -c config/zenoh_router.json
   ```

---

## 底盘通信超时

### 症状

- 日志中出现 `[ERROR] [chassis] robot_XX handshake timeout after 3 retries`。
- 机器人状态在 `online` 与 `offline` 之间频繁切换。
- `chassis_handshake_timeout` 告警频繁触发。

### 原因

1. **网络不稳定**：WiFi 信号弱或存在干扰，导致心跳包丢失。
2. **底盘固件版本不兼容**：固件实现的通信协议与系统预期版本不符。
3. **底盘驱动负载过高**：底盘 CPU 繁忙，无法及时响应心跳请求。
4. **串口/总线通信故障**：工控机与底盘控制器之间的物理接口（USB/RS232/CAN）故障。
5. **超时参数设置过小**：`chassis_handshake_timeout` 和 `max_retries` 参数不适合当前网络环境。

### 解决方案

1. **检查物理连接**：
   ```bash
   # 检查 USB 设备
   lsusb | grep -i serial
   # 检查 CAN 接口
   ip link show can0
   # 检查串口
   dmesg | grep tty
   ```

2. **调整超时参数**：
   ```yaml
   chassis_handshake_timeout: 5000   # 毫秒，从 3000 增大至 5000
   chassis_timeout: 15               # 秒，从 10 增大至 15
   max_retries: 5                    # 从 3 增大至 5
   ```

3. **检查固件版本**：
   ```bash
   ros2 service call /robot_XX/get_firmware_version fleet_msgs/srv/GetFirmwareVersion
   ```
   确保版本与系统兼容。如不匹配，升级底盘固件或更新系统端的通信协议适配层。

4. **降低通信频率**：
   ```yaml
   heartbeat_interval: 500   # 毫秒，从 200 增大至 500
   ```

5. **查看详细超时日志**：
   ```bash
   grep "handshake" /var/log/fleet_system/manager.log | tail -50
   ```
   根据日志中的时间戳和重试序号判断超时发生的具体阶段。

---

## 日志分析

### persist_logger 日志格式

```
[2026-05-04 10:30:00.123] [INFO] [scheduler] [robot_01] Task assigned: task_id=abc-123, start=wp_001, end=wp_020
```

### 关键标签说明

| 标签 | 含义 | 出现场景 |
|------|------|----------|
| `[scheduler]` | 调度器模块 | 任务分配、路径规划、锁管理 |
| `[monitor]` | 监控模块 | 指标采集、告警判定 |
| `[manager]` | 机器人管理器 | 机器人注册、心跳处理、状态变更 |
| `[chassis]` | 底盘通信模块 | 握手、超时、重连 |
| `[deadlock]` | 死锁检测模块 | 死锁检测、死锁解除 |
| `[web]` | Web 后端 | API 请求、WebSocket 事件 |

### 常见日志模式分析

#### 模式 1：正常调度

```
[10:30:00.000] [INFO] [scheduler] Task submitted: task_id=t001
[10:30:00.100] [INFO] [scheduler] [robot_01] Task assigned: task_id=t001
[10:30:00.200] [INFO] [monitor] [robot_01] Task started: task_id=t001
[10:31:15.500] [INFO] [monitor] [robot_01] Task completed: task_id=t001
```
- 正常流程，总耗时约 75 秒。

#### 模式 2：死锁预警

```
[10:30:00.000] [WARN] [deadlock] Potential deadlock detected: robots=[r01,r02,r03], waypoints=[wp_01,wp_02,wp_03]
[10:30:01.000] [WARN] [deadlock] Deadlock confirmed, chain depth=3
[10:30:01.100] [INFO] [deadlock] Breaking deadlock: releasing robot=r03, task=t003
[10:30:02.000] [INFO] [deadlock] Deadlock resolved
```
- 从检测到解除约 2 秒，属正常范围。
- 若此模式频繁出现，需检查地图拓扑（见[死锁反复触发](#死锁反复触发)）。

#### 模式 3：底盘超时

```
[10:30:00.000] [WARN] [manager] [robot_01] Heartbeat missed (1/3)
[10:30:05.000] [WARN] [manager] [robot_01] Heartbeat missed (2/3)
[10:30:10.000] [WARN] [manager] [robot_01] Heartbeat missed (3/3)
[10:30:10.100] [ERROR] [manager] [robot_01] Chassis timeout, marking offline
```
- 三次心跳均丢失，机器人被标记为离线。
- 接下来应检查[底盘通信超时](#底盘通信超时)。

#### 模式 4：任务失败

```
[10:30:00.000] [ERROR] [monitor] [robot_01] Task failed: task_id=t001, reason=nav_timeout
[10:30:00.100] [INFO] [manager] Releasing locks for task_id=t001
[10:30:00.200] [INFO] [scheduler] Retry queued: task_id=t001, retry_count=1
```
- 任务因导航超时失败，锁已释放，自动重试已入队。
- 若 `retry_count` 持续增长，表明目标 waypoint 存在根本性问题。

### 日志分析常用命令

```bash
# 按模块过滤
grep "\[scheduler\]" /var/log/fleet_system/system.log

# 按级别过滤
grep "\[ERROR\]" /var/log/fleet_system/system.log

# 按时间范围过滤
sed -n '/2026-05-04 10:00/,/2026-05-04 11:00/p' /var/log/fleet_system/system.log

# 统计关键词出现频率
grep -o "Deadlock detected" /var/log/fleet_system/system.log | wc -l

# 实时跟踪日志
tail -f /var/log/fleet_system/system.log | grep "\[WARN\]\|\[ERROR\]"
```

---

> **文档版本**: v1.0  
> **最后更新**: 2026-05-04
