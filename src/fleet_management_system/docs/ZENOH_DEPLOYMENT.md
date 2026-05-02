# Zenoh 部署

本文档描述当前仓库中与 Zenoh 相关的实际文件、推荐部署顺序和使用时需要手工核对的配置项。

## 当前仓库内的 Zenoh 文件

### 主机侧 Router 配置

- `docs/zenoh_router_config.yaml`

该文件可直接供 `zenohd` 使用，默认监听 `0.0.0.0:7447`。

### 主机侧 Bridge 配置

- `config/zenoh/host-bridge.json5`

该文件用于 `zenoh-bridge-ros2dds`。当前内容是一个示例，包含如下关键信息：

- `mode: "peer"`
- `connect.endpoints: ["tcp/192.168.1.72:7447"]`
- `plugins.ros2dds.domain: 0`

在真实部署前，请先把 Router 地址改成当前总控机的实际 IP。

## 典型部署拓扑

```text
机器人 ROS2 节点
    │
    │ ros2dds bridge
    ▼
zenoh-bridge-ros2dds (机器人侧)
    │
    │ TCP
    ▼
zenohd (总控机)
    │
    ├── zenoh-bridge-ros2dds (总控机，可选)
    ├── fleet_monitor
    ├── fleet_manager
    └── fleet_web_ui backend
```

## 主机侧启动步骤

### 1. 启动 Router

```bash
cd /home/chen/Documents/works2/fleet_management_system
zenohd -c docs/zenoh_router_config.yaml
```

如果不需要显式配置，也可以直接启动：

```bash
zenohd
```

### 2. 启动主机侧 Bridge

```bash
cd /home/chen/Documents/works2/fleet_management_system
zenoh-bridge-ros2dds -c config/zenoh/host-bridge.json5
```

### 3. 启动 ROS2 业务节点

```bash
cd /home/chen/Documents/works2/fleet_management_system
source install/setup.bash
ros2 run fleet_monitor fleet_monitor_node
ros2 run fleet_manager fleet_manager_node \
  --ros-args -p traffic_map_file:=$PWD/src/fleet_management_system/maps/map0/rmf_map0.yaml
```

## 机器人侧 Bridge 示例

当前仓库没有提交机器人侧专用配置文件，下面给出一份最小模板，使用前需要替换命名空间和 Router 地址：

```json5
{
  mode: "client",
  connect: {
    endpoints: ["tcp/<host-ip>:7447"]
  },
  plugins: {
    ros2dds: {
      domain: 0,
      namespace: "/robot_1",
      allow: {
        publishers: [".*/tf", ".*/odom", ".*/battery_state", ".*/amcl_pose"],
        subscribers: [".*/cmd_vel"],
        action_servers: [".*/navigate_to_pose", ".*/navigate_through_poses"],
        service_servers: [".*"],
        service_clients: [".*"],
        action_clients: [".*"]
      }
    }
  }
}
```

## 命名空间建议

建议为每台机器人分配稳定命名空间，例如：

- `/robot_1`
- `/robot_2`
- `/robot_3`

当前 `fleet_monitor` 会通过 `/tf` 和 ROS 图中的话题名自动发现机器人，不依赖旧文档里提到的 `zenoh_prefix` 启动参数。

## 连接验证

### 检查 Zenoh 侧

确认 Router 已启动且监听正确端口。

### 检查 ROS2 侧

```bash
ros2 topic list | grep -E '/robot_|/fleet_'
ros2 topic echo /fleet_monitor/fleet_status
ros2 node list
```

### 检查 Web 侧

在启动 `server_ros2.py` 后访问：

- `http://localhost:8080/`
- `http://localhost:8080/docs`

## 常见问题

### 机器人看不到

- 检查 Router IP 是否与 `host-bridge.json5` 或机器人侧配置一致。
- 检查 `ROS_DOMAIN_ID` 是否一致。
- 检查网络连通性与防火墙配置。

### 话题存在但状态不更新

- 检查机器人是否真的发布 `/tf`、`/odom`、`/battery_state`。
- 检查 Bridge 过滤规则是否放行了相关话题。

### launch 文件在其他机器上不稳定

当前 `src/fleet_management_system/launch/fleet_system.launch.py` 的默认路径基于 `get_package_share_directory`，已在 colcon install 后可用。若直接从源码目录启动可能需要显式传入参数，详见 KNOWN_ISSUES.md。
