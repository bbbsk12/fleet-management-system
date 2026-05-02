# 构建与运行

本文件只记录当前仓库中已经验证存在的构建对象与启动方式。推荐优先使用手工启动顺序，而不是直接依赖仓库里的脚本包装层。

## 依赖要求

### 必需依赖

- ROS2 Humble 或 Jazzy
- `colcon`
- Python 3
- C++17 编译器
- `yaml-cpp`

### 可选依赖

- `zenohd`
- `zenoh-bridge-ros2dds`
- Node.js 与 npm，用于 Web 前端
- PyQt5、qt-material、pyyaml，用于交通图编辑器

## 构建工作区

```bash
cd /home/chen/Documents/works2/fleet_management_system

source /opt/ros/humble/setup.bash
# 或
# source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
source install/setup.bash
```

如果需要重新开始一轮干净构建，可手工删除 `build`、`install` 和 `log` 后重新执行。

## 构建后验证

```bash
cd /home/chen/Documents/works2/fleet_management_system
source install/setup.bash

colcon list --base-paths src
ros2 pkg list | grep fleet
ls install/fleet_manager/lib/fleet_manager
ls install/fleet_monitor/lib/fleet_monitor
```

当前正常识别到的 ROS2 包应为：

- `fleet_management_system`
- `fleet_manager`
- `fleet_monitor`
- `fleet_msgs`
- `robot_detector`
- `traffic_editor`

## 推荐启动顺序

### 1. 启动 Zenoh Router

单机 ROS2 调试可以跳过此步骤；跨主机部署时建议先启动 Router。

```bash
cd /home/chen/Documents/works2/fleet_management_system
zenohd -c docs/zenoh_router_config.yaml
```

也可以直接运行 `zenohd` 使用默认配置。

### 2. 启动主机侧 Zenoh Bridge

```bash
cd /home/chen/Documents/works2/fleet_management_system
zenoh-bridge-ros2dds -c config/zenoh/host-bridge.json5
```

运行前请先检查 `config/zenoh/host-bridge.json5` 中的 Router 地址是否符合当前网络环境。

### 3. 启动监控节点

```bash
cd /home/chen/Documents/works2/fleet_management_system
source install/setup.bash
ros2 run fleet_monitor fleet_monitor_node
```

### 4. 启动调度节点

调度节点建议显式传入交通图文件。

```bash
cd /home/chen/Documents/works2/fleet_management_system
source install/setup.bash
ros2 run fleet_manager fleet_manager_node \
  --ros-args -p traffic_map_file:=$PWD/maps/map0/rmf_map0.yaml
```

### 5. 启动交通图编辑器

首次使用前安装依赖：

```bash
cd /home/chen/Documents/works2/fleet_management_system
pip install -r src/traffic_editor/requirements.txt
```

推荐启动方式：

```bash
cd /home/chen/Documents/works2/fleet_management_system
source install/setup.bash
ros2 run traffic_editor traffic_editor
```

如需直接调试 Python 入口，现在支持两种方式：

```bash
cd /home/chen/Documents/works2/fleet_management_system
source install/setup.bash
python3 src/traffic_editor/traffic_editor/main_window.py
```

或使用模块方式：

```bash
cd /home/chen/Documents/works2/fleet_management_system
PYTHONPATH=$PWD/src/traffic_editor python3 -m traffic_editor.main_window
```

### 6. 启动 Web UI

先启动后端，再启动前端。

#### 后端

后端依赖 ROS2 环境和已经 source 的工作区；否则会以无 ROS 集成模式启动。

```bash
cd /home/chen/Documents/works2/fleet_management_system
source install/setup.bash

python3 -m venv apps/fleet_web_ui/backend/.venv
source apps/fleet_web_ui/backend/.venv/bin/activate
pip install -r apps/fleet_web_ui/backend/requirements.txt

python apps/fleet_web_ui/backend/server_ros2.py \
  --port 8080 \
  --traffic-map "$PWD/src/fleet_management_system/maps/map0/rmf_map0.yaml"
```

#### 前端

```bash
cd /home/chen/Documents/works2/fleet_management_system/apps/fleet_web_ui/frontend
npm install
npm run dev
```

默认开发端口为 `3000`，并通过 Vite 代理把 `/api` 和 `/ws` 转发到 `8080`。

## 运行验证

### ROS2 侧

```bash
ros2 topic echo /fleet_monitor/fleet_status
ros2 topic echo /fleet_manager/task_status
ros2 service list | grep fleet_manager
```

### Web 侧

- 前端地址：`http://localhost:3000`
- 后端健康检查：`http://localhost:8080/`
- OpenAPI 文档：`http://localhost:8080/docs`

## 启动入口约定

历史上的 `.sh` 启动包装脚本已经从源码中移除，当前只保留本文中的正式启动命令。

如果需要自动化启动，请基于这些正式命令重新实现，而不要恢复旧的临时测试脚本。

当前仍需谨慎使用的入口是：

- `src/fleet_management_system/launch/fleet_system.launch.py`
  Web 后端仍通过命令进程启动，更适合作为调试和集成入口，而不是唯一生产入口。