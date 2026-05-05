# API 参考

---

## REST API

### 基础信息

- **基础 URL**：`http://<SERVER_IP>:8080`
- **数据格式**：JSON
- **字符编码**：UTF-8
- **CORS**：允许所有来源（`*`），可用于跨域开发调试

---

### 系统状态

#### GET /

返回 API 服务基本信息及当前系统概况。

**响应示例：**

```json
{
  "message": "FleetOS API 服务运行中",
  "version": "1.0.0",
  "ros_connected": true,
  "robots_online": 3,
  "waypoints_count": 10
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `message` | string | 服务状态消息 |
| `version` | string | API 版本号 |
| `ros_connected` | bool | ROS2 连接状态 |
| `robots_online` | int | 在线机器人数 |
| `waypoints_count` | int | 航点数量 |

---

#### GET /api/status

获取系统概览状态。

**响应示例：**

```json
{
  "ros_connected": true,
  "total_robots": 5,
  "online_robots": 3,
  "total_tasks": 12,
  "waypoints_count": 10
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `ros_connected` | bool | ROS2 连接状态 |
| `total_robots` | int | 机器人总数 |
| `online_robots` | int | 在线机器人数 |
| `total_tasks` | int | 任务总数 |
| `waypoints_count` | int | 航点数量 |

---

### 机器人管理

#### GET /api/robots

获取所有机器人状态列表。

**响应示例：**

```json
{
  "robots": {
    "AMR-001": {
      "id": "AMR-001",
      "online": true,
      "status": "idle",
      "battery": 78,
      "position": {
        "x": 171.47,
        "y": 163.49,
        "yaw": 0.5,
        "world_x": -5.3,
        "world_y": -2.1
      },
      "current_task": null,
      "nav_status": "idle",
      "connection_status": "online",
      "location_type": "waypoint",
      "current_waypoint": "wp_001",
      "planned_route": [],
      "last_update": "14:30:00"
    }
  }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | string | 机器人唯一标识 |
| `online` | bool | 是否在线 |
| `status` | string | 状态：`offline`、`idle`、`moving`、`arrived`、`failed`、`executing` |
| `battery` | int | 电池电量百分比 (0-100) |
| `position.x` | float | 像素坐标 X |
| `position.y` | float | 像素坐标 Y |
| `position.yaw` | float | 朝向角（弧度） |
| `position.world_x` | float | 世界坐标 X |
| `position.world_y` | float | 世界坐标 Y |
| `current_task` | string | 当前执行的任务 ID，空闲时为 `null` |
| `nav_status` | string | 导航状态 |
| `connection_status` | string | 连接状态：`online`、`offline`、`unknown` |
| `location_type` | string | 位置类型：`waypoint`、`segment`、`unknown` |
| `current_waypoint` | string | 当前所在航点 ID |
| `current_segment` | string | 当前所在航段（例如 `wp_001->wp_002`） |
| `planned_route` | string[] | 规划的路径航点 ID 列表 |
| `last_update` | string | 最后更新时间 |

---

#### GET /api/robots/{robot_id}

获取指定机器人的详细信息。

**参数：**

| 参数 | 位置 | 类型 | 必填 | 说明 |
|------|------|------|------|------|
| `robot_id` | path | string | 是 | 机器人 ID |

**响应：** 同 `GET /api/robots` 中的单个机器人对象。

**错误：**
- `404` — 机器人不存在

---

#### POST /api/robots/{robot_id}/recall

召回机器人。取消该机器人正在执行的任务，使其安全停下并释放调度资源。

**参数：**

| 参数 | 位置 | 类型 | 必填 | 说明 |
|------|------|------|------|------|
| `robot_id` | path | string | 是 | 机器人 ID |

**响应：**

```json
{
  "success": true,
  "robot_id": "AMR-001",
  "task_id": "T-101",
  "cancel": { "success": true }
}
```

**错误：**
- `404` — 机器人不存在

---

#### POST /api/robots/{robot_id}/remove

移除机器人出队。释放该机器人占用的所有航道锁和航线锁，取消任务，清理状态。仅可用于离线机器人检修场景。

**参数：**

| 参数 | 位置 | 类型 | 必填 | 说明 |
|------|------|------|------|------|
| `robot_id` | path | string | 是 | 机器人 ID |

**响应：**

```json
{
  "success": true,
  "message": "机器人已从车队移除"
}
```

**错误：**
- `404` — 机器人不存在
- `504` — ROS2 服务响应超时
- `503` — ROS2 服务未就绪或未连接

---

#### POST /api/robots/{robot_id}/stop

停止单个机器人（Web UI 兼容接口）。

**参数：**

| 参数 | 位置 | 类型 | 必填 | 说明 |
|------|------|------|------|------|
| `robot_id` | path | string | 是 | 机器人 ID |

**响应：**

```json
{
  "success": true
}
```

**错误：**
- `404` — 机器人不存在

---

### 任务管理

#### GET /api/tasks

获取所有任务列表。

**响应示例：**

```json
{
  "tasks": [
    {
      "id": "T-171",
      "waypoint_id": "wp_003",
      "robot_id": "AMR-001",
      "status": "running",
      "priority": 3,
      "task_type": 1,
      "site_code": 0,
      "created_at": "14:20:00"
    }
  ]
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | string | 任务唯一标识 |
| `waypoint_id` | string | 目标航点 ID |
| `robot_id` | string | 分配的机器人 ID，未分配时为 `null` |
| `status` | string | 任务状态（见下方说明） |
| `priority` | int | 优先级（数值越大优先级越高） |
| `task_type` | int | 任务类型：1=CRUISE，2=LOAD，3=UNLOAD，4=SITE_SPECIFIC |
| `site_code` | int | 业务站点代码 |
| `created_at` | string | 创建时间 |

**任务状态枚举：**

| 状态 | 说明 |
|------|------|
| `pending` | 等待分配 |
| `assigned` | 已分配给机器人，等待导航开始 |
| `waiting_fleet` | 等待车队资源（交通管制排队中） |
| `in_progress` | 正在导航 |
| `executing` | 底盘任务执行中（LOAD/UNLOAD 等） |
| `completed` | 已完成 |
| `failed` | 失败 |
| `cancelled` | 已取消 |

**任务类型枚举：**

| 值 | 名称 | 说明 |
|----|------|------|
| `1` | CRUISE | 巡航任务，到达航点即完成 |
| `2` | LOAD | 装载任务，到达后执行装载动作 |
| `3` | UNLOAD | 卸载任务，到达后执行卸载动作 |
| `4` | SITE_SPECIFIC | 自定义站点任务，由 `site_code` 定义具体行为 |

---

#### POST /api/tasks

创建新任务并提交给 fleet_manager 调度。

**请求体：**

```json
{
  "waypoint_id": "wp_003",
  "priority": 1,
  "robot_id": "AMR-001",
  "task_type": 1,
  "site_code": 0
}
```

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `waypoint_id` | string | 是 | — | 目标航点 ID |
| `priority` | int | 否 | `0` | 优先级，数值越大优先级越高 |
| `robot_id` | string | 否 | `""` | 指定执行的机器人 ID。为空时由 fleet_manager 自动选择空闲底盘 |
| `task_type` | int | 否 | `1` | 任务类型：1=CRUISE，2=LOAD，3=UNLOAD，4=SITE_SPECIFIC |
| `site_code` | int | 否 | `0` | 业务站点代码 |

**响应：**

```json
{
  "success": true,
  "task_id": "T-171",
  "task": {
    "id": "T-171",
    "waypoint_id": "wp_003",
    "robot_id": null,
    "status": "pending",
    "priority": 1,
    "task_type": 1,
    "site_code": 0,
    "created_at": "2026-05-04T14:30:00"
  }
}
```

**错误：**
- `503` — ROS2 未连接或任务服务不可用
- `504` — 任务提交超时
- `404` — 指定的机器人不存在
- `400` — 任务提交失败（fleet_manager 返回错误）

---

#### PATCH /api/tasks/{task_id}

手动分配待分配任务到指定机器人。仅支持状态为 `pending` 或 `assigned` 的任务进行重新分配。

**参数：**

| 参数 | 位置 | 类型 | 必填 | 说明 |
|------|------|------|------|------|
| `task_id` | path | string | 是 | 任务 ID |

**请求体：**

```json
{
  "robot_id": "AMR-002"
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `robot_id` | string | 是 | 目标机器人 ID |

**响应：**

```json
{
  "success": true,
  "task": { "id": "T-171", "robot_id": "AMR-002", "status": "assigned", ... }
}
```

**错误：**
- `404` — 任务或机器人不存在
- `400` — 任务状态不允许重新分配

---

#### DELETE /api/tasks/{task_id}

取消任务。优先调用 ROS2 侧的取消服务（真正取消正在执行的 Nav2 goal），若 ROS2 服务不可用则仅标记本地状态。

**参数：**

| 参数 | 位置 | 类型 | 必填 | 说明 |
|------|------|------|------|------|
| `task_id` | path | string | 是 | 任务 ID |

**响应：**

```json
{
  "success": true
}
```

**错误：**
- `404` — 任务不存在
- `504` — 任务取消超时

---

### 控制命令

#### POST /api/emergency_stop

紧急停止指定机器人或全部机器人。通过 WebSocket 广播紧急停止指令给前端。

**请求体：**

```json
{
  "robot_id": "AMR-001"
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `robot_id` | string | 否 | 指定机器人 ID。不提供则停止所有机器人 |

**响应：**

```json
{
  "success": true
}
```

---

### 地图接口

#### GET /api/map/waypoints

获取地图航点列表。

**响应示例：**

```json
{
  "waypoints": [
    {
      "id": "wp_001",
      "name": "wp_001",
      "x": 171.47,
      "y": 163.49,
      "is_charging_station": false,
      "is_parking_spot": false,
      "connections": ["wp_006", "wp_007"]
    }
  ]
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | string | 航点 ID |
| `name` | string | 显示名称 |
| `x` | float | 世界坐标 X |
| `y` | float | 世界坐标 Y |
| `is_charging_station` | bool | 是否为充电站 |
| `is_parking_spot` | bool | 是否为停车位 |
| `connections` | string[] | 连通航点 ID 列表 |

---

#### GET /api/map/info

获取地图基本信息。

**响应：**

```json
{
  "map_data": {
    "map_id": "rmf_map0.yaml",
    "map_name": "交通图",
    "map_resolution": 0.05,
    "map_origin": [-15.3, -21.6, 0.0],
    "waypoints": [...]
  },
  "map_dir": "/path/to/maps/map0",
  "waypoints_count": 10
}
```

---

#### GET /api/map/image

获取地图图片。PGM 格式图片自动转换为 PNG 并以 base64 编码返回。

**响应：**

```json
{
  "image": "data:image/png;base64,iVBORw0KGgo...",
  "filename": "rmf_map.pgm",
  "size": 102400
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `image` | string | Data URL 格式的图片数据 |
| `filename` | string | 原始文件名 |
| `size` | int | 图片文件大小（字节） |

**错误：**
- `404` — 地图图片未配置或文件不存在
- `500` — 读取地图图片失败

---

#### GET /api/map/image/file

获取地图图片文件（直接以文件流形式返回，非 base64 编码）。

**错误：**
- `404` — 地图图片未配置或文件不存在

---

### 系统设置

#### GET /api/settings

获取 WebUI 设置（从服务端持久化文件读取）。

**响应：**

```json
{
  "settings": {
    "ros_domain_id": 0,
    "zenoh_router": "tcp/192.168.1.100:7447",
    "ws_port": 8080
  },
  "settings_file": "/path/to/webui_settings.json"
}
```

---

#### POST /api/settings

更新 WebUI 设置并持久化到文件。部分设置项（如 `ros_domain_id`、`zenoh_router`）需要重启服务方可生效。

**请求体：**

```json
{
  "ros_domain_id": 0,
  "zenoh_router": "tcp/192.168.1.100:7447",
  "ws_port": 8080
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `ros_domain_id` | int | 否 | ROS_DOMAIN_ID |
| `zenoh_router` | string | 否 | Zenoh Router 地址 |
| `ws_port` | int | 否 | WebSocket 端口 |

**响应：**

```json
{
  "success": true,
  "settings": {
    "ros_domain_id": 0,
    "zenoh_router": "tcp/192.168.1.100:7447"
  },
  "restart_required": ["ros_domain_id", "zenoh_router"]
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `restart_required` | string[] | 需要重启服务方可生效的字段列表 |

---

### 日志

#### GET /api/logs

获取系统日志列表。

**参数：**

| 参数 | 位置 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|------|--------|------|
| `limit` | query | int | 否 | `100` | 返回的最大日志条数 |

**响应：**

```json
{
  "logs": [
    {
      "time": "14:30:00",
      "level": "info",
      "message": "创建任务 T-171 -> wp_003",
      "source": "System"
    }
  ]
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `time` | string | 日志时间 |
| `level` | string | 日志级别：`info`、`warning`、`error`、`success` |
| `message` | string | 日志内容 |
| `source` | string | 日志来源 |

---

### WebSocket

#### WS /ws

WebSocket 主端点，处理客户端实时通信。

**连接建立：** 连接成功后自动发送 `init` 消息，包含当前完整系统状态。

**初始消息格式：**

```json
{
  "type": "init",
  "payload": {
    "robots": { ... },
    "tasks": [ ... ],
    "waypoints": [ ... ],
    "map_data": { ... },
    "ros_connected": true
  }
}
```

**客户端发送消息：**

| 消息类型 | Payload | 说明 |
|----------|---------|------|
| `ping` | `{}` | 心跳检测 |
| `get_fleet_status` | `{}` | 获取当前车队状态 |
| `submit_task` | `{"waypoint_id": "...", "priority": 1, "robot_id": "...", "task_type": 1, "site_code": 0}` | 提交新任务 |
| `cancel_task` | `{"task_id": "..."}` | 取消任务 |
| `emergency_stop` | `{"robot_id": "..."}` | 紧急停止 |
| `recall_robot` | `{"robot_id": "..."}` | 召回机器人 |
| `remove_robot` | `{"robot_id": "..."}` | 移除机器人出队 |

**服务端推送消息：**

| 消息类型 | Payload | 说明 |
|----------|---------|------|
| `pong` | `{}` | 心跳回复 |
| `fleet_status` | `{...}` | 车队状态更新（机器人字典） |
| `task_update` | `{"id": "...", "status": "...", ...}` | 任务状态变更 |
| `emergency_stop` | `{"robot_id": "..."}` | 紧急停止指令 |
| `task_created` | `{"id": "...", ...}` | 任务创建成功 |
| `recall_ok` | `{"robot_id": "..."}` | 召回成功 |
| `robot_removed` | `{"robot_id": "...", "result": {...}}` | 机器人已移除 |

---

## ROS2 服务接口

以下 ROS2 服务由 `fleet_manager` 节点提供，可通过 `ros2 service call` 或编写 ROS2 节点调用。

---

### /fleet_manager/submit_task

**类型：** `fleet_msgs/srv/SubmitTask`

提交新任务到调度队列。

**请求：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `waypoint_id` | string | 目标航点 ID |
| `priority` | int32 | 优先级（可选，默认 0） |
| `robot_id` | string | 指定机器人 ID，空字符串表示自动分配 |
| `task_type` | uint8 | 任务类型：1=CRUISE，2=LOAD，3=UNLOAD，4=SITE_SPECIFIC |
| `site_code` | uint32 | 业务站点代码，对 LOAD/UNLOAD/SITE_SPECIFIC 有意义 |

**响应：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 是否提交成功 |
| `task_id` | string | 生成的任务唯一 ID |
| `message` | string | 错误信息（失败时） |

**使用示例：**

```bash
ros2 service call /fleet_manager/submit_task fleet_msgs/srv/SubmitTask \
  '{waypoint_id: "wp_003", priority: 1, robot_id: ""}'
```

---

### /fleet_manager/cancel_task

**类型：** `fleet_msgs/srv/CancelTask`

取消已有任务，同时取消正在执行的 Nav2 导航 goal。

**请求：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | string | 要取消的任务 ID |

**响应：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 是否取消成功 |
| `message` | string | 错误信息（失败时） |

**使用示例：**

```bash
ros2 service call /fleet_manager/cancel_task fleet_msgs/srv/CancelTask \
  '{task_id: "T-171"}'
```

---

### /fleet_manager/get_robot_status

**类型：** `fleet_msgs/srv/GetRobotStatus`

获取指定机器人的状态信息。

**请求：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `robot_id` | string | 机器人 ID。空字符串表示获取所有机器人 |

**响应：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 是否成功 |
| `robots` | RobotStatus[] | 机器人状态列表（见 RobotStatus 数据类型） |
| `message` | string | 错误信息（失败时） |

**使用示例：**

```bash
ros2 service call /fleet_manager/get_robot_status fleet_msgs/srv/GetRobotStatus \
  '{robot_id: "AMR-001"}'
```

---

### /fleet_manager/load_traffic_map

**类型：** `fleet_msgs/srv/LoadTrafficMap`

从文件加载交通地图（运行时热加载，无需重启节点）。

**请求：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `file_path` | string | 交通地图 YAML 文件路径 |

**响应：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 是否加载成功 |
| `map` | TrafficMap | 加载的地图数据（含航点列表） |
| `message` | string | 错误信息（失败时） |

**使用示例：**

```bash
ros2 service call /fleet_manager/load_traffic_map fleet_msgs/srv/LoadTrafficMap \
  '{file_path: "/path/to/maps/map0/rmf_map0.yaml"}'
```

---

### /fleet_manager/save_traffic_map

**类型：** `fleet_msgs/srv/SaveTrafficMap`

将当前交通地图保存到文件。

**请求：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `file_path` | string | 保存路径 |

**响应：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 是否保存成功 |
| `message` | string | 状态或错误信息 |

**使用示例：**

```bash
ros2 service call /fleet_manager/save_traffic_map fleet_msgs/srv/SaveTrafficMap \
  '{file_path: "/path/to/maps/map1/new_map.yaml"}'
```

---

### /fleet_manager/remove_robot

**类型：** `fleet_msgs/srv/RemoveRobot`

从车队中移除机器人。释放该机器人占用的所有航道锁和航线锁，取消任务，清理内部状态。

**请求：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `robot_id` | string | 要移除的机器人 ID |

**响应：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 是否移除成功 |
| `message` | string | 状态或错误信息 |

**使用示例：**

```bash
ros2 service call /fleet_manager/remove_robot fleet_msgs/srv/RemoveRobot \
  '{robot_id: "AMR-005"}'
```

---

## ROS2 话题

### /fleet_monitor/fleet_status

**类型：** `fleet_msgs/msg/FleetStatus`

**发布者：** `fleet_monitor` 节点

**说明：** 车队综合状态话题，包含所有机器人状态、待处理任务列表和系统健康状态。Web 后端订阅此话题（或 `/fleet_manager/fleet_status_traffic`）以获取实时数据。

**字段：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | builtin_interfaces/Time | 时间戳 |
| `robots` | RobotStatus[] | 所有机器人状态列表 |
| `pending_tasks` | TaskInfo[] | 待处理任务列表 |
| `active_tasks` | int32 | 活跃任务总数 |
| `system_status` | string | 系统健康状态：`healthy`、`warning`、`error` |

---

### /fleet_manager/fleet_status_traffic

**类型：** `fleet_msgs/msg/FleetStatus`

**发布者：** `fleet_manager` 节点，频率约 2Hz（随 control timer 回调）

**说明：** 包含交通管制信息的增强版车队状态。在 `/fleet_monitor/fleet_status` 的基础上，增加了当前位置所属航点/航段、规划路径等字段，供 Web 后端和监控面板使用。

---

### /fleet_manager/task_status

**类型：** `fleet_msgs/msg/TaskInfo`

**发布者：** `fleet_manager` 节点

**说明：** 任务状态变更通知话题。每当任务状态发生变化（pending -> assigned -> in_progress -> executing -> completed/failed/cancelled），fleet_manager 会在此话题上发布最新的 TaskInfo 消息。

**字段：** 见下方的 TaskInfo 数据类型。

---

### /fleet_manager/metrics

**类型：** `std_msgs/msg/String`

**发布者：** `fleet_manager` 节点，频率约每 5 秒

**说明：** 运营指标话题，发布 JSON 格式的指标数据，包含死锁打破次数、任务吞吐量等运营统计信息。

---

### /fleet_manager/alerts

**类型：** `std_msgs/msg/String`

**发布者：** `fleet_manager` 节点

**说明：** 系统告警话题，发布调度异常、死锁、超时等重要事件的结构化文本信息，供运维监控系统消费。

---

## 数据类型

### TaskInfo (`fleet_msgs/msg/TaskInfo`)

任务信息消息，在系统各组件间传递任务状态和元数据。

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | string | 任务唯一标识 |
| `waypoint_id` | string | 目标航点 ID |
| `target_pose` | geometry_msgs/Pose | 目标位置（世界坐标） |
| `task_type` | uint8 | 任务类型：1=CRUISE，2=LOAD，3=UNLOAD，4=SITE_SPECIFIC |
| `site_code` | uint32 | 业务站点代码 |
| `status` | string | 任务状态字符串 |
| `assigned_robot_id` | string | 分配的机器人 ID（空表示未分配） |
| `priority` | int32 | 优先级 |
| `created_at` | builtin_interfaces/Time | 创建时间 |
| `started_at` | builtin_interfaces/Time | 开始时间（分配时设置） |
| `completed_at` | builtin_interfaces/Time | 完成时间 |

---

### RobotStatus (`fleet_msgs/msg/RobotStatus`)

机器人状态消息，表示单台机器人的完整运行状态。

| 字段 | 类型 | 说明 |
|------|------|------|
| `robot_id` | string | 机器人唯一标识 |
| `robot_namespace` | string | ROS2 命名空间 |
| `connection_status` | string | 连接状态：`online`、`offline`、`unknown` |
| `battery_percentage` | float32 | 电量百分比 (0.0-100.0) |
| `battery_low` | bool | 电量低标志（< 20% 时为 true） |
| `current_pose` | geometry_msgs/Pose | 当前位置（世界坐标） |
| `current_task_id` | string | 当前任务 ID（空闲时为空） |
| `nav_status` | string | 导航状态：`idle`、`moving`、`arrived`、`failed`、`executing` |
| `location_type` | string | 位置类型：`waypoint`、`segment`、`unknown` |
| `current_waypoint` | string | 所在航点 ID（在航点上时） |
| `current_segment` | string | 所在航段（在航段上时，格式如 `wp_001->wp_002`） |
| `planned_route` | string[] | 规划路径的航点 ID 列表 |
| `last_update` | builtin_interfaces/Time | 最后更新时间 |

---

### FleetStatus (`fleet_msgs/msg/FleetStatus`)

车队综合状态消息。

| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | builtin_interfaces/Time | 时间戳 |
| `robots` | RobotStatus[] | 所有机器人状态列表 |
| `pending_tasks` | TaskInfo[] | 待处理任务列表 |
| `active_tasks` | int32 | 活跃任务总数 |
| `system_status` | string | 系统健康状态：`healthy`、`warning`、`error` |

---

### TrafficMap (`fleet_msgs/msg/TrafficMap`)

交通地图消息。

| 字段 | 类型 | 说明 |
|------|------|------|
| `map_id` | string | 地图标识 |
| `map_name` | string | 地图名称 |
| `map_yaml_path` | string | 原始 YAML 地图文件路径 |
| `waypoints` | Waypoint[] | 所有航点列表 |
| `created_at` | builtin_interfaces/Time | 创建时间 |
| `modified_at` | builtin_interfaces/Time | 最后修改时间 |

---

### Waypoint (`fleet_msgs/msg/Waypoint`)

航点信息消息。

| 字段 | 类型 | 说明 |
|------|------|------|
| `waypoint_id` | string | 航点 ID |
| `name` | string | 可读名称 |
| `pose` | geometry_msgs/Pose | 航点位姿（世界坐标） |
| `connections` | string[] | 连通航点 ID 列表 |
| `is_parking_spot` | bool | 是否为停车位 |
| `is_charging_station` | bool | 是否为充电站 |
| `radius` | float32 | 到达判定半径（米） |

---

### TaskCmd (`fleet_msgs/msg/TaskCmd`)

底盘任务指令消息，由 fleet_manager 发送给底盘（OPi），指示底盘在当前航点执行具体动作。

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | uint64 | 全局唯一任务 ID |
| `waypoint_id` | uint16 | 目标航点索引 |
| `task_type` | uint8 | 任务类型：1=CRUISE，2=LOAD，3=UNLOAD，4=SITE_SPECIFIC |
| `site_code` | uint32 | 业务代码（含义取决于 task_type） |
| `ack` | bool | ACK 标记。为 true 时表示 fleet_manager 确认收到底盘回复 |

---

### TaskFb (`fleet_msgs/msg/TaskFb`)

底盘任务反馈消息，由底盘（OPi）发送给 fleet_manager，报告任务执行状态。

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | uint64 | 任务 ID（必须匹配 TaskCmd.task_id） |
| `status` | uint8 | 任务状态：0=HANDSHAKE_OK，1=COMPLETED，2=ERROR |
| `error_code` | uint8 | 错误码（仅 status=2 时有效） |

**TaskFb 状态枚举：**

| 值 | 名称 | 说明 |
|----|------|------|
| `0` | HANDSHAKE_OK | 底盘收到任务，开始执行 |
| `1` | COMPLETED | 任务执行成功 |
| `2` | ERROR | 任务执行失败 |

**TaskFb 错误码（status=2 时）：**

| 值 | 说明 |
|----|------|
| `0x00` | 未知错误 |
| `0x01` | 参数无效 |
| `0x02` | 串口通信失败（OPi <-> STM32） |
| `0x03` | 执行器无响应（STM32 <-> 执行器） |
| `0x04` | 动作执行失败（如夹爪滑落） |
| `0x05` | 动作执行超时（30 秒） |

---

### LEDTask (`fleet_msgs/msg/LEDTask`)

LED 状态灯控制指令，由 fleet_manager 发送给底盘，控制机器人 LED 灯带显示状态。

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | uint8 | LED 状态：0=WALKING，1=TRAFFIC_WAIT，2=TASK_EXECUTING，3=IDLE |

**LED 状态枚举：**

| 值 | 名称 | 说明 |
|----|------|------|
| `0` | STATE_WALKING | 行走/导航中 |
| `1` | STATE_TRAFFIC_WAIT | 等待交通放行 |
| `2` | STATE_TASK_EXECUTING | 底盘任务执行中 |
| `3` | STATE_IDLE | 空闲 |

---

### LEDStatus (`fleet_msgs/msg/LEDStatus`)

LED 状态灯反馈消息，由底盘发送给 fleet_manager，报告当前实际的 LED 状态。

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | uint8 | 当前实际 LED 状态（0-3，255=DISCONNECTED） |
| `received` | bool | 底盘是否曾收到过 fleet_manager 的 LEDTask 指令 |
