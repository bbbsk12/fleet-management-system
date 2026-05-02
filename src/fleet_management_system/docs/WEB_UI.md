# Web UI

`apps/fleet_web_ui` 是独立于 colcon 的前后端工程，用于展示机器人状态、任务队列和交通图，并通过 REST API 与 WebSocket 提供控制面接口。

## 目录结构

```text
apps/fleet_web_ui/
├── backend/
│   ├── requirements.txt
│   ├── server.py
│   └── server_ros2.py
├── frontend/
│   ├── package.json
│   ├── vite.config.js
│   └── src/
```

## 组件职责

### 后端

- `backend/server_ros2.py`
  当前实际对接 ROS2 的主入口。
- `backend/server.py`
  非 ROS2 集成版本，可视为简化或演示入口，不是当前推荐的生产链路。

### 前端

前端基于 Vue 3、Pinia 和 Vite，主要页面包括：

- Dashboard
- Fleet Monitor
- Task Dispatch
- Map View
- System Logs
- Settings

## 启动要求

### 后端要求

后端若要接入真实 ROS2 数据，启动它的 Shell 必须已经完成：

1. `source /opt/ros/<distro>/setup.bash`
2. `source install/setup.bash`

否则 `server_ros2.py` 会在导入 `fleet_msgs` 或 `rclpy` 失败时退化成无 ROS 模式。

### 前端要求

- Node.js
- npm

## 推荐启动方式

### 启动后端

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

### 启动前端

```bash
cd /home/chen/Documents/works2/fleet_management_system/apps/fleet_web_ui/frontend
npm install
npm run dev
```

## 端口与代理

| 组件 | 默认端口 | 说明 |
| --- | --- | --- |
| Vite 前端 | 3000 | 开发服务器 |
| FastAPI 后端 | 8080 | API 与 WebSocket |

前端开发代理由 `frontend/vite.config.js` 定义：

- `/api` 转发到 `http://localhost:8080`
- `/ws` 转发到 `ws://localhost:8080`

## 后端 API 概览

### 系统状态

- `GET /`
- `GET /api/status`
- `GET /api/logs`
- `GET /api/settings`
- `POST /api/settings`

### 机器人

- `GET /api/robots`
- `GET /api/robots/{robot_id}`
- `POST /api/robots/{robot_id}/recall`
- `POST /api/robots/{robot_id}/stop`
- `POST /api/emergency_stop`

### 地图

- `GET /api/map/info`
- `GET /api/map/waypoints`
- `GET /api/map/image`
- `GET /api/map/image/file`

### 任务

- `GET /api/tasks`
- `POST /api/tasks`
- `PATCH /api/tasks/{task_id}`
- `DELETE /api/tasks/{task_id}`

## WebSocket

WebSocket 入口为 `/ws`，常见消息类型包括：

- 服务端推送：`init`、`fleet_status`、`task_update`、`log`
- 客户端发送：`ping`、`submit_task`、`cancel_task`、`emergency_stop`、`recall_robot`

## 与 ROS2 的连接关系

后端默认订阅：

- `FLEET_STATUS_TOPIC` 环境变量指定的话题
- 默认值为 `/fleet_manager/fleet_status_traffic`

后端同时会调用以下服务客户端：

- `/fleet_manager/submit_task`
- `/fleet_manager/cancel_task`

## 当前注意事项

- 历史上的 `.sh` 启动包装脚本已移除，当前请直接使用本文中的后端与前端命令。
- `backend/requirements.txt` 已包含 `Pillow`，安装后端依赖时会一并安装地图图片转换所需库。
- 浏览器侧 WebSocket 地址会结合本地设置中的 `ws_port` 与当前 host 拼接，如果端口或反向代理策略发生变化，需要同步调整前端设置。