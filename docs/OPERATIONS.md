# 运维手册

## 目录

1. [日常监控](#日常监控)
2. [Web 控制台使用](#web-控制台使用)
3. [交通图编辑](#交通图编辑)
4. [任务管理](#任务管理)
5. [底盘管理](#底盘管理)
6. [告警处理](#告警处理)
7. [性能调优](#性能调优)
8. [日志管理](#日志管理)
9. [备份恢复](#备份恢复)

---

## 日常监控

系统通过 ROS2 话题对外暴露运行时指标。监控系统可订阅以下话题以获取实时状态：

| 指标 | 话题名称 | 类型 | 说明 |
|------|----------|------|------|
| 在线机器人数量 | `robots_online` | `std_msgs/Int32` | 当前正在心跳的机器人数量 |
| 待处理任务数 | `tasks_pending` | `std_msgs/Int32` | 处于 `pending` 状态的任务队列长度 |
| 活跃任务数 | `tasks_active` | `std_msgs/Int32` | 正在执行中的任务数量 |
| 已完成任务数 | `tasks_completed` | `std_msgs/Int32` | 累计完成任务总数 |
| 失败任务数 | `tasks_failed` | `std_msgs/Int32` | 累计失败任务总数 |
| 死锁解除次数 | `deadlock_breaks` | `std_msgs/Int32` | 死锁检测及自动解除的累计次数 |

### 监控建议

- 通过 `ros2 topic echo <topic_name>` 实时查看指标数据。
- 使用 `ros2 topic hz <topic_name>` 检查话题发布频率是否正常（通常应不低于 1 Hz）。
- 建议将以上指标接入 Prometheus + Grafana 等外部监控平台，建立告警规则：
  - `robots_online` 低于预期阈值时触发告警。
  - `tasks_pending` 持续堆积超过上限时触发告警。
  - `deadlock_breaks` 短时内激增表示地图拓扑可能存在严重缺陷。

---

## Web 控制台使用

Web 控制台提供基于浏览器的管理系统，默认地址为 `http://<host>:8000`。以下为各页面功能说明。

### Dashboard

系统概览面板，展示：
- 机器人总数与在线数。
- 任务统计（待处理 / 执行中 / 已完成 / 失败）。
- 死锁事件实时计数。
- 系统资源使用概况（CPU、内存）。

### Fleet Monitor

车队监控视图，以列表形式展示每台机器人的详细信息：
- 机器人 ID、状态（idle / moving / charging / offline）、当前位置、目标点。
- 机器人当前任务 ID 及任务状态。
- 最后心跳时间戳。
- 支持按状态筛选与搜索。

### Map View

交通图可视化页面：
- 在 2D Canvas 上渲染路网拓扑（waypoints、lanes、zones）。
- 实时显示机器人位置与运动轨迹。
- 染色标识阻塞路段与死锁区域。
- 支持缩放、平移、点击查看节点详情。

### Task Dispatch

任务下发面板：
- 选择任务类型（`delivery`、`patrol`、`custom`）。
- 设置起点 / 终点 waypoint。
- 配置优先级（0-255，数值越大优先级越高）。
- 指定固定机器人（可选）。
- 查看当前任务队列及历史记录。

### Settings

系统参数配置页面：
- 调度参数：`waypoint_radius`、`sched_interval`、`ghost_lock_ttl`。
- 死锁参数：`deadlock_timeout`、`chain_depth`。
- 日志级别切换。
- 交通图文件上传与切换。

### System Logs

系统日志实时查看页面：
- 支持按日志级别（DEBUG / INFO / WARN / ERROR / FATAL）过滤。
- 支持按模块（scheduler / monitor / manager / chassis）过滤。
- 支持关键词搜索。
- 日志自动滚动与暂停。

---

## 交通图编辑

交通图是系统运行的核心配置文件，建议使用 `traffic_editor` GUI 工具进行编辑。

### Waypoint 管理

- **添加 waypoint**：在画布上右键选择 "Add Waypoint"，设置唯一名称（如 `wp_001`）。
- **编辑 waypoint**：左键双击 waypoint 打开属性面板，可修改名称、位置坐标（x, y, yaw）、类型（normal / charger / depot / elevator）。
- **删除 waypoint**：选中 waypoint 后按 Delete 键或右键选择删除。删除前需确保该 waypoint 无关联的 lane。

### Connection 编辑

- **添加 lane**：选中起点 waypoint，按住 Ctrl 并点击终点 waypoint，右键选择 "Add Lane"。
- **编辑 lane 属性**：双击 lane 可设置方向（bidirectional / forward / backward）、速度限制、是否可跨越。
- **添加 zone**：框选多个 waypoint，右键选择 "Add Zone"，可定义充电区、待命区、装载区等。

### 地图加载与保存

- **加载地图**：在 traffic_editor 中通过 `File > Open` 选择 `.yaml` 格式的交通图文件。
- **保存地图**：编辑完成后通过 `File > Save` 或 `Ctrl+S` 保存。建议每次编辑前先备份原文件。
- **导入系统**：将编辑完成的 `.yaml` 文件放置于 `fleet_manager/config/` 目录下，或在 Web 控制台 Settings 页面上传。

### 编辑规范

- waypoint 命名遵循 `{区域缩写}_{序号}` 格式，如 `wh_001`（仓库）、`ld_001`（装货区）。
- lane 方向应与实际交通规则一致，避免出现无法通行的单向环路。
- 充电站 waypoint 需设置类型为 `charger`，并在 properties 中注明充电器 ID。
- 编辑完成后运行 `traffic_editor --validate <map_file>` 检查拓扑完整性。

---

## 任务管理

### 提交任务

通过 ROS2 服务接口或 Web 控制台提交任务：

```bash
ros2 service call /submit_task fleet_msgs/srv/SubmitTask "{task_type: 'delivery', start: 'wp_001', end: 'wp_020', priority: 100}"
```

参数说明：
- `task_type`：任务类型，可选 `delivery`、`patrol`、`custom`。
- `start` / `end`：起点与终点 waypoint 名称。
- `priority`：优先级，范围 0-255，默认 0。
- `robot_id`：指定执行机器人（可选），留空则由调度器自动分配。

### 优先级管理

- 高优先级任务会插队至队列前端。
- 正在执行的任务不受优先级变化影响；新任务仅在下一轮调度中竞争。
- 建议紧急任务使用优先级 200-255，普通任务使用 0-100。

### 固定机器人分配

在提交任务时指定 `robot_id` 参数，调度器会将任务直接分配给指定机器人。若该机器人忙，任务将进入专属等待队列。

### 取消任务

```bash
ros2 service call /cancel_task fleet_msgs/srv/CancelTask "{task_id: 'task_uuid_here'}"
```

- 仅可取消 `pending` 状态的任务。
- 已进入 `in_progress` 的任务需触发 emergency stop 后手动处理。
- 取消操作会清除该任务在调度器中的所有占位锁。

### 紧急停止

```bash
ros2 service call /emergency_stop fleet_msgs/srv/EmergencyStop "{robot_id: 'robot_01'}"
```

- 立即停止指定机器人的当前导航目标。
- 机器人将原地制动，不自动恢复。
- 需通过 Web 控制台或服务接口手动发送恢复指令 `resume` 以继续。

---

## 底盘管理

### 添加机器人

通过 Web 控制台 Fleet Monitor 页面的 "Add Robot" 按钮，或调用以下服务接口：

```bash
ros2 service call /register_robot fleet_msgs/srv/RegisterRobot "{robot_id: 'robot_01', chassis_type: 'agv', firmware_version: '2.1.0'}"
```

注册流程：
1. 系统向新机器人发送 Handshake 请求。
2. 机器人回复握手确认后，状态标记为 `online`。
3. 调度器开始为该机器人分配任务。

### 移除机器人

```bash
ros2 service call /unregister_robot fleet_msgs/srv/UnregisterRobot "{robot_id: 'robot_01'}"
```

移除前系统会自动处理：
- 若机器人正在执行任务，先触发任务中断。
- 释放该机器人占用的所有 waypoint 锁。
- 从调度器的活跃机器人列表中移除。

### 离线处理

当机器人的心跳话题超过 `chassis_timeout`（默认 10 秒）未更新时，系统自动将其标记为 `offline`：

1. **自动处理**：调度器将离线机器人的待执行任务重新分配至其他可用机器人（若 `auto_relocate` 开启）。
2. **手动干预**：运维人员需检查机器人物理状态，重启底盘驱动后，系统自动发起 Handshake 恢复连接。
3. **强制下线**：若机器人损坏无法恢复，使用 UnregisterRobot 接口将其从系统中彻底移除。

---

## 告警处理

### Ghost Lock 过期

- **触发条件**：waypoint 锁持有者超过 `ghost_lock_ttl`（默认 30 秒）未释放锁，且未发送心跳续期。
- **系统行为**：自动释放过期锁，允许其他机器人抢占该 waypoint。
- **运维建议**：
  - 若频繁出现 ghost lock，检查对应机器人是否因导航异常卡在路径上。
  - 适当调整 `ghost_lock_ttl` 值，避免在网络延迟大的环境中频繁误判。

### 死锁解除

- **触发条件**：调度器检测到机器人之间形成循环等待（deadlock cycle），超过 `deadlock_timeout`（默认 5 秒）未解除。
- **系统行为**：自动选择链中优先级最低的机器人，取消其当前任务并释放占用的锁，打破死锁链。
- **运维建议**：
  - 查看 `deadlock_breaks` 指标，若频繁触发说明地图拓扑存在不良环路。
  - 检查 deadlock 链日志，定位瓶颈节点，优化 lane 方向或添加旁路路径。

### 任务失败

- **触发条件**：机器人无法到达目标 waypoint、导航超时、底盘通信中断等。
- **系统行为**：任务状态标记为 `failed`，释放所有占用的资源；若 `auto_retry` 开启，自动重试。
- **运维建议**：
  - 通过 `tasks_failed` 话题监控失败率。
  - 查看 `fleet_monitor` 日志中 `task_failure` 标签，定位失败原因。
  - 对于因临时障碍导致的失败，手动在 Web 控制台重新提交任务。

### 底盘超时

- **触发条件**：底盘心跳超过 `chassis_timeout`（默认 10 秒）未收到。
- **系统行为**：触发断连重试机制，最多重试 `max_retries`（默认 3 次）次；重试均失败后标记为 `offline`。
- **运维建议**：
  - 检查底盘与工控机的物理网络连接。
  - 核实底盘固件版本是否与系统兼容。
  - 查看 `chassis_handshake_timeout` 日志确认超时节点。

---

## 性能调优

| 参数 | 默认值 | 说明 | 调优建议 |
|------|--------|------|----------|
| `waypoint_radius` | 0.5 m | waypoint 到达判定半径 | 场地狭窄可减小至 0.3 m，开阔场地可增大至 0.8 m |
| `sched_interval` | 1000 ms | 调度器主循环间隔 | 机器人数量少（<10）可减小至 500 ms；数量多（>50）可增大至 2000 ms |
| `ghost_lock_ttl` | 30 s | 幽灵锁租约有效期 | 网络不稳定时可增大至 60 s |
| `deadlock_timeout` | 5 s | 死锁判定超时时间 | 复杂拓扑建议增大至 10 s，避免误判 |
| `chain_depth` | 5 | 死锁链最大检测深度 | 机器人密度高可增大至 10 |
| `chassis_timeout` | 10 s | 底盘心跳超时阈值 | 弱网络环境可增大至 20 s |
| `max_retries` | 3 | 底盘重连最大重试次数 | 根据网络可靠性在 2-5 范围调整 |
| `purge_finished` | true | 是否自动清理已完成任务 | 保持开启以释放内存 |
| `repair_queue_limit` | 100 | 修理队列上限 | 内存充足可增大至 500 |

### 调优原则

- 先分析瓶颈（CPU 高 / 任务积压 / 死锁频繁），再有针对地调整。
- 每次仅修改 1-2 个参数，观察至少 30 分钟运行效果。
- 记录每次参数变更前后的指标数据，便于回归对比。

---

## 日志管理

### persist_logger 输出格式

日志行格式如下：

```
[2026-05-04 10:30:00.123] [INFO] [scheduler] [robot_01] Task assigned: task_id=abc-123, start=wp_001, end=wp_020
```

字段说明：

| 字段 | 说明 |
|------|------|
| `[timestamp]` | 日志时间，格式 `YYYY-MM-DD HH:mm:SS.mmm` |
| `[level]` | 日志级别：DEBUG / INFO / WARN / ERROR / FATAL |
| `[module]` | 模块名称：scheduler / monitor / manager / chassis / web |
| `[tag]` | 关联的机器人 ID 或任务 ID |
| 消息体 | 结构化日志，使用 `key=value` 格式 |

### 日志轮转

系统默认使用 `logrotate` 进行日志轮转（适用于 Linux）：

```text
/var/log/fleet_system/*.log {
    daily
    rotate 30
    compress
    delaycompress
    missingok
    notifempty
    copytruncate
}
```

- 保留最近 30 天日志。
- 每日轮转，旧日志压缩存储。
- 日志文件达到 100 MB 时自动分割。

### 日志级别管理

- 生产环境建议日志级别为 `INFO`。
- 故障排查时临时切换至 `DEBUG`：

  ```bash
  ros2 param set /fleet_manager log_level DEBUG
  ros2 param set /fleet_monitor log_level DEBUG
  ```

- `DEBUG` 级别会产生大量日志，排查完成后及时切回 `INFO`。

### Verbose 模式

启动时添加 `--ros-args --log-level debug` 参数可启用完整调试日志：

```bash
ros2 run fleet_manager fleet_manager --ros-args --log-level debug
```

---

## 备份恢复

### 交通图备份

交通图文件是系统最重要的配置之一，建议定期备份：

- **自动备份**：系统每次启动时自动将当前加载的交通图文件备份至 `backups/maps/` 目录，文件名为 `map_YYYYMMDD_HHmmss.yaml`。
- **手动备份**：

  ```bash
  cp config/traffic_map.yaml backups/maps/map_manual_$(date +%Y%m%d).yaml
  ```

- **恢复操作**：

  ```bash
  cp backups/maps/map_20260504.yaml config/traffic_map.yaml
  ros2 service call /reload_map fleet_msgs/srv/ReloadMap
  ```

### 设置备份

运行参数配置文件位于 `config/settings.yaml`，包含所有用户自定义参数：

- **手动备份**：

  ```bash
  cp config/settings.yaml backups/settings/settings_$(date +%Y%m%d).yaml
  ```

- **恢复操作**：将备份文件覆盖 `config/settings.yaml` 后重启系统，或通过 Web 控制台 Settings 页面的 "Import Config" 功能动态加载。

### PostgreSQL 数据库备份

若使用持久化存储且启用了 PostgreSQL：

```bash
pg_dump -U fleet_user fleet_db > backups/db/fleet_db_$(date +%Y%m%d).sql
```

### 备份策略建议

| 备份类型 | 频率 | 保留时间 |
|----------|------|----------|
| 交通图文件 | 每次编辑后 | 永久 |
| 参数设置 | 每次变更后 | 6 个月 |
| 数据库 | 每日 | 30 天 |
| 系统日志 | 每日 | 30 天 |

---

> **文档版本**: v1.0  
> **最后更新**: 2026-05-04
