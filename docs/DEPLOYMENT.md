# 部署指南

## 环境要求

### 操作系统

- Ubuntu 22.04 LTS (推荐)
- 其他 Linux 发行版亦可，需自行适配依赖安装方式

### ROS2 中间件

| 组件 | 版本 | 说明 |
|------|------|------|
| ROS2 | Humble Hawksbill | 长期支持版本，经充分测试 |
| Nav2 | Humble 对应版本 | 导航栈，负责路径规划与底盘控制 |
| Zenoh Bridge | 1.0.0+ | 跨机器 ROS2 通信桥梁 |
| CycloneDDS / Fast-DDS | 任意 | DDS 实现，建议使用 CycloneDDS |

### 运行环境

| 运行时 | 最低版本 | 用途 |
|--------|----------|------|
| Python | 3.10+ | ROS2 节点与 Web 后端 |
| Node.js | 18+ | 前端构建与开发 |
| npm | 9+ | 前端依赖管理 |

### 硬件要求 (服务端)

| 资源 | 最低配置 | 推荐配置 |
|------|----------|----------|
| CPU | 4 核 | 8 核 |
| 内存 | 4 GB | 8 GB |
| 磁盘 | 20 GB | 50 GB (含日志存储) |
| 网络 | 千兆以太网 | 千兆以太网 |

---

## 编译构建

### 1. 安装 ROS2 Humble

```bash
# 设置软件源
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null
sudo apt update

# 安装 ROS2 Humble Desktop
sudo apt install -y ros-humble-desktop

# 安装编译工具与依赖
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# 初始化 rosdep
sudo rosdep init
rosdep update
```

### 2. 安装 Nav2 及相关依赖

```bash
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3-gazebo
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

### 3. 安装 Zenoh Bridge

```bash
# 方式一：从 Eclipse Zenoh 官方仓库安装
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list
sudo apt update
sudo apt install -y zenoh-bridge-ros2dds

# 方式二：从源码编译 (需 Rust 工具链)
# cargo install zenoh-bridge-ros2dds
```

### 4. 创建工作空间并编译

```bash
# 创建工作空间
mkdir -p ~/fleet_ws/src
cd ~/fleet_ws

# 克隆项目代码
git clone <repository-url> src/fleet_management_system

# 安装 ROS2 依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 环境配置
source install/setup.bash
echo "source ~/fleet_ws/install/setup.bash" >> ~/.bashrc
```

### 5. 编译前端 Web UI

```bash
cd src/fleet_management_system/apps/fleet_web_ui/frontend
npm install
npm run build
# 编译产物位于 dist/ 目录
```

---

## Zenoh 部署

### 系统架构

Zenoh 在系统中的角色是实现跨机器 ROS2 通信的桥梁。典型拓扑如下：

```
┌─────────────────── 服务端 (总控电脑) ───────────────────┐
│  zenohd (Router, tcp/0.0.0.0:7447)                     │
│       │                                                 │
│  zenoh-bridge-ros2dds (Client 模式)                     │
│       │                                                 │
│  fleet_manager / fleet_monitor / Web Backend            │
└─────────────────────────────────────────────────────────┘
                              │  TCP (跨机器)
                              ▼
┌─────────────────── 机器人端 ────────────────────────────┐
│  zenoh-bridge-ros2dds (Client 模式, 连接服务端 7447)   │
│       │                                                 │
│  Nav2 / robot_localization / 底盘驱动                    │
└─────────────────────────────────────────────────────────┘
```

### 服务端 Zenoh Router 配置

文件位置：`~/zenoh_router_config.yaml`

```yaml
mode: "peer"
listen:
  endpoints:
    - "tcp/0.0.0.0:7447"
connect:
  endpoints: []
transport:
  unicast:
    accept_timeout: 10000
    accept_pending: 100
    max_sessions: 1000
  link:
    protocols: ["tcp"]
routing:
  router_peers_failover_brokering: true
  gossip:
    autoconnect: ["router", "peer"]
logging:
  level: "info"
```

### 服务端 Zenoh Bridge 配置

文件位置：`src/fleet_management_system/config/zenoh/host-bridge.json5`

```json5
{
  mode: "client",
  connect: {
    endpoints: ["tcp/127.0.0.1:7447"]
  },
  scouting: {
    multicast: { enabled: false },
    gossip: { enabled: false }
  },
  plugins: {
    ros2dds: {
      domain: 0,
      ros_automatic_discovery_range: "LOCALHOST",
      allow: {
        publishers: [".*"],
        subscribers: [".*"],
        service_servers: [".*"],
        service_clients: [".*"],
        action_servers: [".*"],
        action_clients: [".*"]
      }
    }
  }
}
```

### 机器人端 Zenoh Bridge 配置

```json5
{
  mode: "client",
  connect: {
    // 替换为服务端 IP 地址
    endpoints: ["tcp/<SERVER_IP>:7447"]
  },
  scouting: {
    multicast: { enabled: false },
    gossip: { enabled: false }
  },
  plugins: {
    ros2dds: {
      domain: 0,
      ros_automatic_discovery_range: "LOCALHOST",
      allow: {
        publishers: [".*"],
        subscribers: [".*"],
        service_servers: [".*"],
        service_clients: [".*"],
        action_servers: [".*"],
        action_clients: [".*"]
      }
    }
  }
}
```

### 启动 Zenoh

```bash
# 服务端：启动 Zenoh Router
zenohd -c ~/zenoh_router_config.yaml

# 服务端：启动 Zenoh Bridge（由 launch 文件自动启动）
# 确保 ROS_DOMAIN_ID 与环境一致

# 机器人端：启动 Zenoh Bridge
export ROS_DOMAIN_ID=0
export ZENOH_ROUTER=tcp/<SERVER_IP>:7447
zenoh-bridge-ros2dds -c robot-bridge.json5
```

---

## 配置文件

### Launch 参数说明

启动文件：`src/fleet_management_system/launch/fleet_system.launch.py`

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `traffic_map_file` | string | `maps/map0/rmf_map0.yaml` | 交通地图文件路径（YAML 格式，含航点与连接关系） |
| `use_web` | bool | `true` | 是否启动 Web 上位机后端 |
| `web_port` | int | `8080` | Web 后端 HTTP 端口 |
| `zenoh_bridge_config` | string | `config/zenoh/host-bridge.json5` | Zenoh Bridge 配置文件路径 |
| `ros_domain_id` | int | `0` | ROS_DOMAIN_ID，确保所有节点在同一域 |
| `zenoh_router` | string | `""` | Zenoh Router 地址，留空则由 bridge 配置决定 |
| `scheduler_verbose_log` | bool | `false` | 是否启用调度详细日志 |
| `persist_log_enabled` | bool | `true` | 是否启用结构化落盘日志 |
| `persist_log_dir` | string | `test_logs/` | 持久化日志输出目录 |
| `persist_log_verbose_info` | bool | `false` | 是否在持久化日志中包含 INFO 级别信息 |
| `waypoint_acceptance_radius` | double | `0.5` | 机器人到达航点的接受半径（米） |
| `traffic_segment_lateral_max` | double | `1.25` | 航段横向偏差最大阈值（米） |
| `chassis_handshake_timeout_sec` | double | `5.0` | 底盘任务握手超时（秒） |
| `chassis_exec_timeout_sec` | double | `30.0` | 底盘任务执行超时（秒） |
| `chassis_max_retries` | int | `3` | 底盘握手重试次数 |

### Fleet Manager 内部参数

这些参数定义在 `fleet_manager_node.hpp` 中，可通过 ROS2 参数动态设置：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `scheduler_interval_sec` | double | `1.0` | 调度主循环周期（秒） |
| `ghost_lock_ttl_sec` | double | `120.0` | 幽灵锁自动过期时间（秒） |
| `retry_base_sec` | double | `1.0` | 导航重试退避基础时间（秒） |
| `retry_max_count` | int | `5` | 单航点最大重试次数 |
| `nav_stuck_timeout_sec` | double | `20.0` | 导航卡住判定超时（秒） |
| `nav_absolute_timeout_sec` | double | `45.0` | 导航绝对超时（秒） |
| `monitor_stale_timeout_sec` | double | `4.0` | 监控数据陈旧超时（秒） |
| `deadlock_timeout_sec` | double | `10.0` | 死锁持续判定超时（秒）后触发打破 |
| `max_task_retry_cycles` | int | `5` | 任务最大保留性回队轮数 |

---

## 多机部署

### 机器人端配置

每台机器人需运行以下组件：

1. **底盘驱动**：对接底层电机和控制器的 ROS2 节点
2. **Nav2 导航栈**：提供全局/局部路径规划和底盘控制
3. **Zenoh Bridge**：将机器人上的 ROS2 话题/服务桥接到服务端

机器人端启动示例（`robot_bringup.sh`）：

```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/fleet_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export ZENOH_ROUTER=tcp/<SERVER_IP>:7447

# 启动 Nav2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false &

# 启动机器人状态发布节点
ros2 run my_robot_driver robot_driver_node &

# 启动 Zenoh Bridge
zenoh-bridge-ros2dds
```

### 服务端配置

服务端运行以下组件：

1. **Zenoh Router** (`zenohd`)：作为通信中枢
2. **Zenoh Bridge** (`zenoh-bridge-ros2dds`)：桥接本地 DDS 和 Zenoh 网络
3. **Fleet Monitor**：监控每台机器人的在线状态和位置
4. **Fleet Manager**：调度中枢，负责任务分配、交通管制和死锁检测
5. **Web Backend**：REST API + WebSocket 服务
6. **Web Frontend**：Vue.js 编制的管理界面

服务端启动通过统一的 launch 文件完成。

### 网络要求

- 服务端需有固定 IP 地址
- 机器人端需能够通过 TCP 连接到服务端的 7447 端口
- 同一局域网下延迟应 < 10ms，丢包率 < 0.1%
- 机器人数量超过 10 台时建议使用有线网络或专用 5GHz Wi-Fi

---

## 地图准备

### 交通地图 YAML 格式

交通地图文件定义了航点、连接关系和地图元数据。以 `maps/map0/rmf_map0.yaml` 为例：

```yaml
map_id: rmf_map0.yaml
map_name: 交通图
map_image: rmf_map.pgm          # 栅格地图图片文件（PGM 格式）
map_yaml_path: rmf_map.yaml     # Nav2 标准地图 YAML 文件路径
map_resolution: 0.05            # 地图分辨率（米/像素）
map_origin: [-15.3, -21.6, 0]   # 地图原点世界坐标 [x, y, yaw]
waypoints:
  - id: wp_001                  # 航点唯一标识
    name: wp_001                # 航点显示名称
    position:                   # 世界坐标（单位：米）
      x: 171.47
      y: 163.49
      z: 0.0
    radius: 0.5                 # 到达判定半径
    is_charging_station: false  # 是否为充电站
    is_parking_spot: false      # 是否为停车位
    connections:                # 连通航点 ID 列表
      - wp_006
      - wp_007
  - id: wp_005
    name: robot0_init
    position:
      x: 259.76
      y: 259.49
      z: 0.0
    radius: 0.5
    is_charging_station: true
    is_parking_spot: true
    connections:
      - wp_002
  # ...更多航点
```

### 航点属性说明

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `id` | string | 是 | 航点唯一标识，全局不可重复 |
| `name` | string | 否 | 可读名称，默认同 `id` |
| `position.x` | float | 是 | 世界坐标 X |
| `position.y` | float | 是 | 世界坐标 Y |
| `position.z` | float | 是 | 世界坐标 Z（通常为 0） |
| `radius` | float | 否 | 到达判定半径（米），默认由全局 `waypoint_acceptance_radius` 决定 |
| `is_charging_station` | bool | 否 | 标记为充电站 |
| `is_parking_spot` | bool | 否 | 标记为停车位 |
| `connections` | string[] | 是 | 可达航点 ID 列表，定义图的拓扑连接 |

### 栅格地图

系统需要两个地图文件配合使用：

1. **Nav2 占用栅格地图**：用于机器人导航避障的标准 `occupancy_grid`（PGM + YAML）
2. **交通地图**：在栅格地图基础上抽象出的拓扑航点图（自定义 YAML 格式）

使用 `traffic_editor` 工具（位于 `src/traffic_editor/`）可图形化编辑交通地图：

```bash
# 启动交通地图编辑器
python3 src/traffic_editor/scripts/traffic_editor
```

---

## 启动流程

### 完整启动步骤

#### 步骤 1：启动 Zenoh Router

在服务端执行：

```bash
source /opt/ros/humble/setup.bash
zenohd -c ~/zenoh_router_config.yaml
```

#### 步骤 2：启动机器人端

在各机器人上执行：

```bash
# 启动 Nav2 导航栈
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false

# 启动 Zenoh Bridge（连接服务端）
export ROS_DOMAIN_ID=0
export ZENOH_ROUTER=tcp/<SERVER_IP>:7447
zenoh-bridge-ros2dds
```

#### 步骤 3：启动服务端 ROS2 系统

在服务端新终端执行：

```bash
source ~/fleet_ws/install/setup.bash
ros2 launch fleet_management_system fleet_system.launch.py
```

launch 文件会自动依次启动：
  - 等待 Zenoh Router 就绪（检测 tcp/127.0.0.1:7447）
  - 启动 `zenoh-bridge-ros2dds`
  - 启动 `fleet_monitor` 节点
  - 启动 `fleet_manager` 节点
  - 启动 Web 后端（FastAPI，默认端口 8080）

#### 步骤 4：启动 Web 前端

```bash
cd src/fleet_management_system/apps/fleet_web_ui/frontend

# 开发模式
npm run dev

# 或使用预构建的生产版本
npx serve dist
```

浏览器访问 `http://<SERVER_IP>:8080` 即可打开管理界面。

### 快速诊断命令

```bash
# 检查节点是否正常运行
ros2 node list
# 预期输出：
#   /fleet_monitor
#   /fleet_manager

# 检查车队状态话题
ros2 topic echo /fleet_monitor/fleet_status

# 检查任务状态话题
ros2 topic echo /fleet_manager/task_status

# 检查服务列表
ros2 service list | grep fleet_manager
```

---

## Docker 部署

### Docker Compose 示例

创建 `docker-compose.yml`：

```yaml
version: "3.8"

services:
  zenohd:
    image: eclipse/zenoh:latest
    container_name: fleet-zenohd
    network_mode: host
    command: ["-c", "/config/zenoh_router_config.yaml"]
    volumes:
      - ./zenoh_router_config.yaml:/config/zenoh_router_config.yaml
    restart: unless-stopped

  zenoh-bridge:
    image: eclipse/zenoh-bridge-ros2dds:latest
    container_name: fleet-zenoh-bridge
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
    command: ["-c", "/config/host-bridge.json5"]
    volumes:
      - ./host-bridge.json5:/config/host-bridge.json5
    restart: unless-stopped
    depends_on:
      - zenohd

  fleet-core:
    build:
      context: .
      dockerfile: Dockerfile.fleet
    container_name: fleet-core
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
    volumes:
      - ./maps:/workspace/install/fleet_management_system/share/fleet_management_system/maps
      - ./test_logs:/workspace/test_logs
      - ./runtime:/workspace/runtime
    command: >
      ros2 launch fleet_management_system fleet_system.launch.py
      traffic_map_file:=/workspace/install/fleet_management_system/share/fleet_management_system/maps/map0/rmf_map0.yaml
    depends_on:
      - zenoh-bridge

  web-backend:
    build:
      context: .
      dockerfile: Dockerfile.web
    container_name: fleet-web-backend
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
      - FLEET_STATUS_TOPIC=/fleet_manager/fleet_status_traffic
    volumes:
      - ./runtime:/workspace/runtime
    depends_on:
      - fleet-core
```

### Dockerfile.fleet

```dockerfile
FROM ros:humble-ros-base

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-gazebo-ros-pkgs \
    yaml-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY src/ src/
RUN colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]
```

### Dockerfile.web

```dockerfile
FROM python:3.11-slim

RUN apt update && apt install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY src/ src/
COPY apps/ apps/

# 安装 Python 依赖
RUN pip install fastapi uvicorn websockets pydantic python-multipart Pillow pyyaml

EXPOSE 8080

CMD ["python3", "apps/fleet_web_ui/backend/server_ros2.py", "--port", "8080"]
```

---

## 生产环境检查清单

### 部署前检查

- [ ] Ubuntu 22.04 系统已更新到最新补丁
- [ ] ROS2 Humble 已正确安装并可正常运行 `ros2 run demo_nodes_cpp talker`
- [ ] Nav2 在单机仿真环境中已通过测试
- [ ] Zenoh Bridge 版本 >= 1.0.0
- [ ] 所有机器人均可 Ping 通服务端 IP
- [ ] 服务端 7447 端口已开放（防火墙规则）
- [ ] 交通地图文件已就位且格式正确
- [ ] 所有航点的 `connections` 字段已正确配置（双向连接）
- [ ] 每台机器人的初始位置在交通地图上有对应的航点
- [ ] 前端已编译（`npm run build`）且 dist 目录存在

### 运行时监控

- [ ] `ros2 node list` 输出包含 `fleet_manager` 和 `fleet_monitor`
- [ ] `ros2 topic echo /fleet_monitor/fleet_status` 能看到机器人状态更新
- [ ] Web 界面可通过 `http://<SERVER_IP>:8080` 正常访问
- [ ] 机器人在 Web 界面上显示为 online 状态
- [ ] 可通过 Web 界面创建任务并成功分配
- [ ] 持久化日志已写入 `test_logs/` 目录

### 性能调优

- [ ] 多机器人场景下适当增大 `ghost_lock_ttl_sec`（建议 120-300 秒）
- [ ] 高密度航点场景适当减小 `scheduler_interval_sec`（建议 0.5-1.0 秒）
- [ ] 机器人数量超过 10 台时考虑调整 `monitor_stale_timeout_sec` 为 2-3 秒
- [ ] 定期清理 `test_logs/` 目录中过期的日志文件（建议使用 logrotate）
- [ ] 检查 `webui_settings.json` 中的 `ros_domain_id` 是否与环境一致

### 故障排除

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 机器人显示 offline | Zenoh Bridge 未连接 | 检查机器人端 `zenoh-bridge-ros2dds` 进程是否运行 |
| Web 界面无数据 | ROS_DOMAIN_ID 不一致 | 确认所有组件的 ROS_DOMAIN_ID 相同 |
| 任务无法创建 | fleet_manager 未启动 | 执行 `ros2 node list` 确认节点存在 |
| 导航卡住不动 | Nav2 初始位姿未设置 | 通过 `ros2 topic pub /initialpose` 设置初始位姿 |
| 死锁无法自动恢复 | `deadlock_timeout_sec` 过小 | 适当增大死锁检测超时 |
| 航点无法到达 | 连接关系配置错误 | 检查 `connections` 字段是否双向配置 |
| Zenoh 连接失败 | 防火墙阻挡 7447 端口 | 确认服务端 `ufw` 或 `iptables` 已放行 7447 |
