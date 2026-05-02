# 系统架构

## 工作区结构

当前工作区是一个标准的 ROS2/colcon 仓库。ROS2 包源码位于 `src/`，而 Web 上位机作为独立工程位于 `apps/fleet_web_ui`。元包 `src/fleet_management_system/` 负责统一安装 `launch/`、`config/`、`maps/`、`docs/`、`scripts/`（以及 Web 后端脚本）到 share 目录。

```text
fleet_management_system/
├── src/
│   ├── fleet_management_system/     # 元包：launch、config、maps、docs、scripts
│   │   ├── launch/
│   │   ├── config/zenoh/
│   │   ├── maps/map0/
│   │   ├── docs/
│   │   └── scripts/
│   ├── fleet_manager/
│   ├── fleet_monitor/
│   ├── fleet_msgs/
│   ├── robot_detector/
│   ├── traffic_editor/
├── apps/
│   └── fleet_web_ui/
│       ├── backend/
│       └── frontend/
│           └── src/
│               ├── locales/    # i18n 翻译文件
│               ├── stores/     # Pinia 状态管理
│               └── ...
├── build/
├── install/
└── log/
```

## 包级职责

| 目录 | 技术栈 | 构建类型 | 作用 | 入口 |
| --- | --- | --- | --- | --- |
| `src/fleet_management_system` | CMake | `ament_cmake` | 元包，安装 `launch/`、`config/`、`maps/`、`docs/`、`scripts/` | `launch/fleet_system.launch.py` |
| `src/fleet_msgs` | ROSIDL | `ament_cmake` | 定义消息和服务接口 | 无可执行程序 |
| `src/fleet_monitor` | C++17 | `ament_cmake` | 自动发现机器人并汇总全局状态 | `fleet_monitor_node` |
| `src/fleet_manager` | C++17 | `ament_cmake` | 任务调度、交通图加载、冲突处理 | `fleet_manager_node` |
| `src/robot_detector` | Python | `ament_python` | 机器人发现与诊断工具 | `robot_detector`、`detect_robots`、`zenoh_echo` |
| `src/traffic_editor` | Python + PyQt5 | `ament_python` | 图形化交通图编辑器 | `traffic_editor` |
| `apps/fleet_web_ui` | Vue 3 + FastAPI | 独立工程 | Web 控制台与接口层 | `frontend`、`backend/server_ros2.py` |

## 运行链路

### 1. 机器人侧

机器人通过 ROS2 话题、服务和动作提供底盘状态与导航能力。若跨主机部署，则通常通过 Zenoh Bridge 将这些接口暴露到总控机。

### 2. 监控层

`fleet_monitor` 监听 `/tf` 并扫描当前 ROS 图中的机器人命名空间，聚合机器人的在线状态、电量、位姿和任务信息，然后发布统一车队状态：

- 发布：`/fleet_monitor/fleet_status`
- 服务：`~/get_robot_status`

### 3. 调度层

`fleet_manager` 订阅监控层汇总结果，结合交通图完成任务分配、导航下发、冲突规避和任务状态广播：

- 订阅：`/fleet_monitor/fleet_status`
- 发布：`/fleet_manager/fleet_status_traffic`
- 发布：`~/task_status`
- 服务：`~/submit_task`
- 服务：`~/cancel_task`
- 服务：`~/get_robot_status`
- 服务：`~/load_traffic_map`
- 服务：`~/save_traffic_map`

### 4. 可视化与操作层

- `traffic_editor` 通过服务与 `fleet_manager` 交互，用于编辑和保存交通图。
- `fleet_web_ui/backend/server_ros2.py` 订阅 ROS2 状态并对外暴露 REST API 与 WebSocket。
- `fleet_web_ui/frontend` 通过 `/api/*` 和 `/ws` 与后端通信，展示机器人、任务和地图信息。

## 接口层

### 消息定义

`fleet_msgs` 当前提供以下消息：

- `RobotStatus.msg`
- `FleetStatus.msg`
- `TaskInfo.msg`
- `TrafficMap.msg`
- `Waypoint.msg`

其中 `RobotStatus` 额外包含 `location_type`、`current_waypoint`、`current_segment` 和 `planned_route` 等字段，Web UI 的地图视图依赖这些字段渲染路径和位置。

### 服务定义

`fleet_msgs` 当前提供以下服务：

- `SubmitTask.srv`
- `CancelTask.srv`
- `GetRobotStatus.srv`
- `LoadTrafficMap.srv`
- `SaveTrafficMap.srv`

## 地图与配置

### 地图文件

示例地图位于 `src/fleet_management_system/maps/map0`：

- `rmf_map.yaml`：占据栅格地图配置
- `rmf_map.pgm`：地图图像
- `rmf_map0.yaml`：交通图与航点连接关系

### Zenoh 配置

当前仓库包含两类 Zenoh 配置：

- `src/fleet_management_system/config/zenoh/host-bridge.json5`：主机侧 `zenoh-bridge-ros2dds` 示例配置
- `src/fleet_management_system/docs/zenoh_router_config.yaml`：主机侧 `zenohd` 示例配置

## 当前确认的范围边界

- `apps/fleet_web_ui` 目前不参与 colcon 构建。
- 元包 `fleet_management_system` 负责安装 `launch/`、`config/`、`maps/`、`docs/`、`scripts/` 到 share 目录。