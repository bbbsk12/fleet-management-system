# 交通图编辑器

交通图编辑器由 `src/traffic_editor` 提供，是一个基于 PyQt5 的图形化工具，用于加载底图、编辑航点、连接路线，并通过 ROS2 服务与 `fleet_manager` 交互。

## 目录结构

```text
src/traffic_editor/
├── package.xml
├── requirements.txt
├── setup.py
└── traffic_editor/
    ├── main_window.py
    └── traffic_map_widget.py
```

## 依赖安装

```bash
cd /home/chen/Documents/works2/fleet_management_system
pip install -r src/traffic_editor/requirements.txt
```

依赖包括：

- PyQt5
- qt-material
- pyyaml

## 推荐启动方式

```bash
cd /home/chen/Documents/works2/fleet_management_system
source install/setup.bash
ros2 run traffic_editor traffic_editor
```

## 调试启动方式

如果需要跳过 `ros2 run` 直接调试 Python 入口，可直接执行脚本：

```bash
cd /home/chen/Documents/works2/fleet_management_system
source install/setup.bash
python3 src/traffic_editor/traffic_editor/main_window.py
```

也可以使用模块方式：

```bash
cd /home/chen/Documents/works2/fleet_management_system
PYTHONPATH=$PWD/src/traffic_editor python3 -m traffic_editor.main_window
```

## 与 ROS2 的交互

编辑器会创建以下服务客户端：

- `/fleet_manager/load_traffic_map`
- `/fleet_manager/save_traffic_map`
- `/fleet_manager/submit_task`

因此在使用加载、保存或提交任务功能前，建议先启动 `fleet_manager`。

## 典型工作流

### 1. 加载地图底图

使用地图 YAML 或现有交通图文件作为编辑基础。当前仓库内置示例位于：

- `maps/map0/rmf_map.yaml`
- `maps/map0/rmf_map0.yaml`

### 2. 编辑航点

当前版本支持的交互模式包括：

- 平移地图
- 添加航点
- 连接航点
- 编辑航点属性

### 3. 设置属性

航点可包含以下属性：

- `id`
- `name`
- `connections`
- `is_parking_spot`
- `is_charging_station`
- `radius`

### 4. 保存交通图

保存后应生成与 `rmf_map0.yaml` 类似的交通图文件，供 `fleet_manager` 读取。

### 5. 提交导航任务

编辑器可把选中的目标航点提交给 `fleet_manager`，由调度层决定分配与执行。

## 文件格式

交通图示例文件 `maps/map0/rmf_map0.yaml` 包含：

- 地图标识信息
- `map_image`
- `map_yaml_path`
- 航点列表
- 航点连接关系

当前示例和编辑器保存逻辑都会优先使用相对 `map_yaml_path`，以便地图目录整体移动后仍可用。

栅格地图 YAML `src/fleet_management_system/maps/map0/rmf_map.yaml` 包含：

- `image`
- `resolution`
- `origin`
- `occupied_thresh`
- `free_thresh`

## 当前注意事项

- 编辑器本身是 ROS2 包，推荐通过 `ros2 run` 启动。
- 若未安装 `qt-material`，界面仍可能启动，但主题效果会退化。
- 如果 `fleet_manager` 未运行，加载、保存和提交任务相关功能会失败。