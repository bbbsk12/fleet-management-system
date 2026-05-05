# 车队调度管理系统 (Fleet Management System)

基于 ROS2 的多机器人车队调度系统，支持任务分配、交通管制、冲突解决、死锁检测、链式撤退以及 Web 可视化控制台。

---

## 功能特性

- **任务调度** — 优先级队列 + 最近距离贪心分配 + FCFS 门控机制，支持多类型任务（CRUISE / LOAD / UNLOAD / SITE_SPECIFIC）
- **交通管制** — 基于 Zone 的区域占用管理（zone_locks + reservations），双重保护保障多车安全共行
- **瓶颈冲突检测** — 对向路径冲突检测，自动 defer 低优先级任务，防止迎面碰撞
- **死锁检测** — 阻塞图 + DFS 环检测，自动选择 victim 并物理移走，支持 relocate 到最近空闲航点
- **链式撤退** — 多车互锁时的递归撤退链构建（深度 5 层），有序协调释放死锁
- **幽灵锁** — 离线底盘的 Zone 占用保留（Ghost Lock TTL），防止因底盘掉线导致的碰撞风险
- **自动导航** — 集成 Nav2（NavigateToPose action），支持逐航点路径导航、卡住重试、绝对超时回队
- **底盘任务执行** — 通过 ROS2 Topic 与底盘通信握手，支持 LOAD/UNLOAD 等自动化任务，含超时重试
- **自动驶离** — 任务完成后自动将空闲底盘从单出口死胡同航点移出
- **Web 控制台** — FastAPI + WebSocket 实时可视化，支持地图显示、任务管理、机器人召回、移除出队
- **机器人自动发现** — 通过 /tf 话题及话题列表扫描自动发现并监控机器人
- **交通图编辑** — 内置 Qt 图形化编辑器，支持航点增删改、连接管理、地图关联
- **Zenoh 通信桥接** — 支持跨网络 Zenoh 路由，实现远程机器人接入
- **结构化持久日志** — 关键事件落盘，支持运维诊断

---

## 系统架构

```
                      +---------------------+
                      |   Web 控制台前端      |
                      | (fleet_web_ui)       |
                      +----------+----------+
                                 |
                       REST API / WebSocket
                                 |
                      +----------+----------+
                      |   Web 后端 (FastAPI)  |
                      | server_ros2.py       |
                      +----------+----------+
                                 |
                      ROS2 Service Client
                                 |
+--------------------------------+-----------------------------------+
|                          ROS2 Master Node                          |
|                                                                    |
|  +-------------------+    +------------------------------------+   |
|  |  Fleet Monitor     |    |         Fleet Manager              |   |
|  |  (fleet_monitor)   |    |  +----------------+  +----------+  |   |
|  |                    |    |  | TaskScheduler   |  | Traffic  |  |   |
|  |  - TF 发现机器人    |    |  |  - 优先级队列    |  | Manager  |  |   |
|  |  - 在线心跳判定     |    |  |  - 贪心分配      |  |  - BFS   |  |   |
|  |  - 位姿更新         |    |  |  - FCFS 门控    |  |  寻路    |  |   |
|  |  - 电池监控         |    |  +----------------+  |  - Dijk  |  |   |
|  |                    |    |                        |  stra    |  |   |
|  |  ---> /fleet_      |    |  +----------------+  | 加权     |  |   |
|  |  monitor/fleet_    |    |  | Occupancy      |  +----------+  |   |
|  |  status            |    |  | Manager        |                |   |
|  +-------------------+    |  |  - zone_locks   |                |   |
|                            |  |  - reservations |                |   |
|  +-------------------+    |  |  - ghost_locks  |                |   |
|  |  Robot Detector    |    |  +----------------+                |   |
|  |  (robot_detector)  |    +------------------------------------+   |
|  |  - Zenoh 扫描      |               |                             |
|  +-------------------+          Nav2 Action                          |
|                                  /robotX/navigate_to_pose           |
+--------------------------------+-----------------------------------+
                                 |
                    +------------+------------+
                    |    Robot 1 | Robot 2 | ... |
                    |  (底盘 + Nav2 + STM32)   |
                    +-------------------------+

                    +-------------------------+
                    |  Zenoh Bridge           |
                    |  (跨网络远程机器人接入)    |
                    +-------------------------+
```

---

## 快速开始

### 环境依赖

- **ROS2 Humble** (Ubuntu 22.04)
- **Nav2** (Navigation2 stack)
- **Zenoh** (可选，用于跨网络通信)
- Python 3.10+ (Web 后端)
- Qt5 / PyQt5 (traffic_editor)

### 编译

```bash
cd fleet_management_system
colcon build --symlink-install
source install/setup.bash
```

### 启动

```bash
# 完整系统启动（含 Zenoh Bridge、Fleet Monitor、Fleet Manager、Web 后端）
ros2 launch fleet_management_system fleet_system.launch.py

# 仅启动核心调度节点
ros2 run fleet_manager fleet_manager_node --ros-args \
  -p traffic_map_file:=/path/to/map.yaml

# 启动 Web 后端（独立模式）
python3 apps/fleet_web_ui/backend/server_ros2.py --port 8080
```

### 提交测试任务

```bash
# 通过 ROS2 服务提交任务
ros2 service call /fleet_manager/submit_task fleet_msgs/srv/SubmitTask \
  "{waypoint_id: 'wp_001', priority: 1, task_type: 1}"

# 查看车队状态
ros2 topic echo /fleet_manager/fleet_status_traffic
```

---

## 目录结构

```
fleet_management_system/
├── apps/
│   └── fleet_web_ui/           # Web 可视化控制台
│       ├── backend/             # FastAPI 后端 + ROS2 桥接
│       └── frontend/            # 前端资源
├── docs/                        # 技术文档
│   ├── ARCHITECTURE.md          # 系统架构文档
│   ├── API.md                   # API 参考
│   ├── DEPLOYMENT.md            # 部署指南
│   ├── DEVELOPMENT.md           # 开发指南
│   ├── OPERATIONS.md            # 运维手册
│   └── TROUBLESHOOTING.md       # 故障排除
├── src/
│   ├── fleet_msgs/              # ROS2 消息/服务/动作定义
│   │   ├── msg/                 # FleetStatus, RobotStatus, TaskInfo, TaskCmd, etc.
│   │   └── srv/                 # SubmitTask, CancelTask, GetRobotStatus, etc.
│   ├── fleet_monitor/           # 车队监控节点 (C++)
│   ├── fleet_manager/           # 调度管理核心 (C++)
│   │   ├── src/                 # 节点主循环、调度、导航、交通管制、占用管理
│   │   └── include/fleet_manager/
│   ├── fleet_management_system/ # 启动文件、脚本、地图、配置
│   │   ├── launch/              # ROS2 launch 文件
│   │   ├── maps/                # 交通地图文件
│   │   ├── scripts/             # 诊断/压力测试/仿真脚本
│   │   └── config/              # Zenoh 桥接配置
│   ├── robot_detector/          # Zenoh 机器人发现 (Python)
│   └── traffic_editor/          # 交通图编辑器 (PyQt5)
├── runtime/                     # 运行时数据目录
├── test_logs/                   # 持久化日志输出目录
├── build/                       # 编译输出
└── install/                     # 安装输出
```

---

## 文档索引

| 文档 | 说明 |
|------|------|
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | 系统架构详解、数据流、算法说明 |
| [API.md](docs/API.md) | ROS2 接口 / REST API 参考 |
| [DEPLOYMENT.md](docs/DEPLOYMENT.md) | 部署配置与安装指南 |
| [DEVELOPMENT.md](docs/DEVELOPMENT.md) | 开发环境搭建与贡献指南 |
| [OPERATIONS.md](docs/OPERATIONS.md) | 运维监控与参数调优 |
| [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | 常见问题与故障排查 |

---

## 许可证

内部项目。未经授权不得分发。
