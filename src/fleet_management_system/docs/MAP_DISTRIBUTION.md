# 地图文件与分发

本项目当前同时使用两类地图文件：占据栅格地图和交通图。部署时需要明确这两类文件的职责，避免只同步其中一部分导致导航与调度语义不一致。

## 当前仓库中的地图文件

示例地图位于 `maps/map0`：

- `rmf_map.pgm`
  栅格地图图像。
- `rmf_map.yaml`
  Nav2 或地图服务常用的地图描述文件，引用 `rmf_map.pgm`。
- `rmf_map0.yaml`
  交通图文件，包含航点、连线、停车位、充电位等调度语义信息。

## 两类地图的职责

### 占据栅格地图

用于定位、避障和路径规划相关组件，典型字段包括：

- `image`
- `resolution`
- `origin`
- `occupied_thresh`
- `free_thresh`

### 交通图

用于车队级任务调度与路线语义建模，典型字段包括：

- `map_id`
- `map_name`
- `map_yaml_path`
- `waypoints`
- `connections`
- `is_parking_spot`
- `is_charging_station`

## 当前推荐的地图组织方式

建议保持每个地图目录同时包含：

```text
maps/<map_name>/
├── rmf_map.pgm
├── rmf_map.yaml
└── rmf_map0.yaml
```

这样可以把底图、导航地图和交通图的关系固定在同一目录内，便于同步与版本管理。

## 推荐分发策略

当前更推荐使用“本地副本 + 手工同步或脚本同步”的方式，而不是把地图依赖完全交给运行时网络共享。

原因是：

- 机器人端 Nav2 启动时通常希望本地就能拿到地图。
- 交通图与占据栅格地图应保持同版本同步。
- 网络临时抖动不应阻断机器人启动。

## 推荐同步方式

### 单机器人同步

```bash
rsync -avz maps/map0/ robot1@192.168.1.101:~/maps/map0/
```

### 多机器人同步

```bash
for robot in robot1@192.168.1.101 robot2@192.168.1.102; do
  rsync -avz maps/map0/ "$robot":~/maps/map0/
done
```

### 使用变量的同步脚本

```bash
#!/bin/bash
set -e

MAP_SRC="$(pwd)/src/fleet_management_system/maps/map0"
ROBOTS=("robot1@192.168.1.101" "robot2@192.168.1.102")

for robot in "${ROBOTS[@]}"; do
  ssh "$robot" "mkdir -p ~/maps/map0"
  rsync -avz --delete "$MAP_SRC/" "$robot":~/maps/map0/
done
```

## 交通图更新流程

### 1. 编辑

通过 `traffic_editor` 修改航点和连接关系。

### 2. 保存

保存新的交通图文件，例如 `rmf_map0.yaml`。

### 3. 验证

在总控机上使用 `fleet_manager` 加载新交通图，确认航点和任务下发正常。

### 4. 分发

把同一版本的占据栅格地图与交通图一起同步到机器人侧。

## 当前需要注意的现实问题

### `map_yaml_path` 建议使用相对路径

交通图中的 `map_yaml_path` 建议始终写为相对当前交通图文件的路径，例如 `rmf_map.yaml`。

当前仓库示例已经使用相对路径，交通图编辑器在保存时也会优先输出相对路径，以提高目录迁移和分发时的稳定性。

### Web 地图预览依赖图片转换

Web 后端会读取地图图片并在 `.pgm` 场景下尝试转为 PNG，因此运行 Web UI 时需要安装 `backend/requirements.txt` 中的依赖，其中已包含 `Pillow`。

## 发布建议

- 小规模部署优先使用 `rsync` 或 `scp` 同步。
- 修改交通图时同时保留文件版本记录。
- 地图目录不要只同步 `rmf_map0.yaml`，应连同 `rmf_map.yaml` 与 `rmf_map.pgm` 一起同步。
