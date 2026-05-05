# 开发指南

## 目录

1. [环境搭建](#环境搭建)
2. [项目结构详解](#项目结构详解)
3. [构建与测试](#构建与测试)
4. [代码规范](#代码规范)
5. [添加新功能](#添加新功能)
6. [调试技巧](#调试技巧)
7. [单元测试](#单元测试)

---

## 环境搭建

### 系统要求

- **操作系统**: Ubuntu 22.04 LTS (推荐) / Ubuntu 24.04 LTS
- **ROS2 发行版**: Humble (Ubuntu 22.04) / Iron (Ubuntu 24.04)
- **编译器**: GCC 11+ (C++17) / Clang 14+
- **Python**: 3.10+
- **Node.js**: 18 LTS+ (Web 前端)

### ROS2 工作区设置

```bash
# 安装 ROS2 Humble (Ubuntu 22.04)
sudo apt install ros-humble-desktop

# 安装构建工具及依赖
sudo apt install python3-colcon-common-extensions python3-vcstool \
                 python3-rosdep python3-pip

# 创建工作区
mkdir -p ~/fleet_ws/src
cd ~/fleet_ws

# 克隆仓库
git clone <repository_url> src/fleet_management_system

# 安装系统依赖
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 构建
colcon build --symlink-install

# 设置环境变量
echo "source ~/fleet_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 依赖清单

| 包名 | 版本要求 | 来源 |
|------|----------|------|
| `rclcpp` | ROS2 Humble | apt |
| `rclpy` | ROS2 Humble | apt |
| `nav2_msgs` | Humble | apt |
| `tf2_geometry_msgs` | Humble | apt |
| `ament_cmake` | Humble | apt |
| `nlohmann-json` | 3.10+ | apt / vcpkg |
| `yaml-cpp` | 0.7+ | apt |
| `libcurl-dev` | 任意 | apt |
| `nodejs` | 18+ | nodesource |
| `npm` | 9+ | nodesource |
| `python3-flask` | 2.0+ | pip |
| `python3-websocket` | 10+ | pip |

### Web 前端依赖

```bash
cd src/fleet_management_system/fleet_web_ui/frontend
npm install
```

---

## 项目结构详解

```
fleet_management_system/
├── fleet_msgs/                  # 自定义消息与服务定义
│   └── msg/
│       ├── RobotStatus.msg      # 机器人状态消息
│       ├── Task.msg             # 任务数据消息
│       └── DeadlockEvent.msg    # 死锁事件消息
│   └── srv/
│       ├── SubmitTask.srv       # 提交任务服务
│       ├── CancelTask.srv       # 取消任务服务
│       ├── EmergencyStop.srv    # 紧急停止服务
│       ├── RegisterRobot.srv    # 注册机器人服务
│       └── UnregisterRobot.srv  # 注销机器人服务
│   └── CMakeLists.txt
│   └── package.xml
│
├── fleet_manager/               # 核心调度器（C++）
│   ├── src/
│   │   ├── main.cpp             # 入口
│   │   ├── scheduler.cpp/h      # 调度算法核心
│   │   ├── task_queue.cpp/h     # 任务队列管理
│   │   ├── robot_manager.cpp/h  # 机器人状态管理
│   │   ├── map_manager.cpp/h    # 交通图管理
│   │   ├── deadlock_detector.cpp/h # 死锁检测与解除
│   │   └── lock_manager.cpp/h   # Waypoint 锁管理
│   ├── config/                  # 配置文件
│   │   ├── traffic_map.yaml     # 交通图
│   │   └── settings.yaml        # 运行参数
│   ├── CMakeLists.txt
│   └── package.xml
│
├── fleet_monitor/               # 监控节点（Python）
│   ├── fleet_monitor/
│   │   ├── __init__.py
│   │   ├── monitor_node.py      # 监控主节点
│   │   ├── metrics_collector.py # 指标收集
│   │   ├── alert_manager.py     # 告警管理
│   │   └── data_logger.py       # 数据持久化
│   ├── resource/                # ROS2 资源
│   ├── setup.py
│   └── package.xml
│
├── fleet_web_ui/                # Web 管理界面
│   ├── backend/                 # Python Flask 后端
│   │   ├── app.py               # Flask 应用入口
│   │   ├── routes/
│   │   │   ├── dashboard.py     # Dashboard API
│   │   │   ├── fleet.py         # 车队管理 API
│   │   │   ├── tasks.py         # 任务管理 API
│   │   │   ├── map.py           # 地图管理 API
│   │   │   └── settings.py      # 设置 API
│   │   ├── websocket_handler.py # WebSocket 实时通信
│   │   └── requirements.txt
│   └── frontend/                # React 前端
│       ├── src/
│       │   ├── components/
│       │   │   ├── Dashboard/
│       │   │   ├── FleetMonitor/
│       │   │   ├── MapView/
│       │   │   ├── TaskDispatch/
│       │   │   ├── Settings/
│       │   │   └── SystemLogs/
│       │   ├── hooks/           # React Hooks
│       │   ├── services/        # API 调用封装
│       │   └── utils/           # 工具函数
│       ├── package.json
│       └── vite.config.ts
│
├── fleet_bridge/                # Zenoh-DDS 桥接层（可选）
│   ├── src/
│   │   └── zenoh_bridge.cpp
│   ├── CMakeLists.txt
│   └── package.json
│
├── docs/                        # 文档
│   ├── OPERATIONS.md
│   ├── DEVELOPMENT.md
│   └── TROUBLESHOOTING.md
│
└── scripts/                     # 运维脚本
    ├── start_all.sh
    ├── stop_all.sh
    ├── backup.sh
    └── monitor_check.sh
```

---

## 构建与测试

### 构建

```bash
# 全量构建
cd ~/fleet_ws
colcon build --symlink-install

# 仅构建特定包
colcon build --packages-select fleet_manager

# 构建时启用测试编译
colcon build --cmake-args -DBUILD_TESTING=ON

# 构建且不安装测试（加速迭代）
colcon build --cmake-args -DBUILD_TESTING=OFF
```

### 测试

```bash
# 运行所有测试
colcon test

# 运行特定包测试
colcon test --packages-select fleet_manager

# 查看测试结果
colcon test-result --verbose
```

### Lint 检查

```bash
# C++ 代码检查 (ament_cpplint)
colcon test --packages-select fleet_manager --ctest-args -R cpplint

# Python 代码检查 (ament_flake8)
colcon test --packages-select fleet_monitor --ctest-args -R flake8

# 手动运行 linter
ament_cpplint src/fleet_manager/src/
ament_flake8 src/fleet_monitor/
```

### 构建优化

```bash
# 并行编译（根据 CPU 核心数调整）
colcon build --parallel-workers 8

# 只编译变更文件（增量编译）
colcon build --symlink-install --continue-on-error
```

---

## 代码规范

### 通用原则

- 所有代码文件和注释统一使用 UTF-8 编码。
- 行尾使用 LF，不包含尾随空格。
- 文件末尾保留一个空行。

### C++ 规范

- **语言标准**: C++17。
- **命名约定**:
  - 类名 / 结构体: `PascalCase`（如 `TaskQueue`、`RobotManager`）。
  - 函数名 / 方法: `snake_case`（如 `submit_task()`、`detect_deadlock()`）。
  - 成员变量: `snake_case_`（末尾下划线，如 `task_queue_`）。
  - 常量: `kPascalCase` 或全大写 `UPPER_SNAKE_CASE`。
  - 文件命名: `snake_case.cpp` / `snake_case.h`。
- **头文件规范**:
  - 使用 `#pragma once` 而非宏守卫。
  - 头文件应自包含（包含自身所需的所有头文件）。
  - 前置声明优先于不必要的 `#include`。
- **注释风格**:
  - 公共 API 使用 Doxygen 风格注释。
  - 复杂逻辑需添加行内注释说明意图。
- **禁止使用**:
  - 禁止使用 `using namespace std;`。
  - 禁止使用 C 风格指针（除非与 C 库交互）。
  - 禁止使用 `malloc` / `free`。

```cpp
// 示例：头文件注释模板
/**
 * @brief 调度器核心类，负责任务分配与路径规划
 *
 * Scheduler 管理任务队列、机器人状态和交通图，
 * 在每一轮调度循环中为待处理任务分配最优机器人。
 */
class Scheduler {
public:
  explicit Scheduler(const rclcpp::NodeOptions& options);
  ~Scheduler();

  /**
   * @brief 提交新任务
   * @param task 任务数据结构
   * @return 是否成功入队
   */
  bool submit_task(const Task& task);

private:
  TaskQueue task_queue_;
  RobotManager robot_manager_;
};
```

### Python 规范

- **语言标准**: Python 3.10+。
- **命名约定**:
  - 类名: `PascalCase`。
  - 函数 / 方法: `snake_case`。
  - 模块 / 文件: `snake_case.py`。
  - 私有方法: `_leading_underscore`。
- **类型注解**: 所有公共函数必须包含类型注解。

```python
# 示例：Python 代码风格
from typing import Optional


class MetricsCollector:
    """指标收集器，聚合系统运行时数据。"""

    def __init__(self, node_name: str = "metrics_collector") -> None:
        self._node_name = node_name
        self._metrics: dict[str, int] = {}

    def collect(self) -> dict[str, int]:
        """收集当前指标快照。"""
        return dict(self._metrics)
```

---

## 添加新功能

### 添加新的 ROS2 消息/服务 (fleet_msgs)

1. 在 `fleet_msgs/msg/` 下创建 `.msg` 文件，或在 `fleet_msgs/srv/` 下创建 `.srv` 文件。
2. 编辑 `fleet_msgs/CMakeLists.txt`，将新文件加入 `rosidl_generate_interfaces` 列表。
3. 编辑 `fleet_msgs/package.xml`，确认依赖已声明。
4. 编译验证：

   ```bash
   colcon build --packages-select fleet_msgs
   ```

5. 在其他包中引入并测试：

   ```cpp
   #include "fleet_msgs/msg/new_message.hpp"
   #include "fleet_msgs/srv/new_service.hpp"
   ```

### 添加新的调度策略 (fleet_manager)

1. 在 `fleet_manager/src/` 下创建新的调度策略类文件（如 `custom_strategy.cpp` / `custom_strategy.h`）。
2. 继承或实现调度器策略接口：

   ```cpp
   class CustomStrategy {
   public:
     virtual Robot select_robot(const Task& task, const std::vector<Robot>& robots) = 0;
     virtual Path plan_path(const std::string& start, const std::string& end) = 0;
   };
   ```

3. 在主调度器 `scheduler.cpp` 中注入新策略。
4. 在 `settings.yaml` 中添加策略选择参数。
5. 添加对应的单元测试。

### 添加新的监控指标 (fleet_monitor)

1. 在 `fleet_monitor/fleet_monitor/metrics_collector.py` 中添加指标采集逻辑：

   ```python
   class MetricsCollector:
       def collect_custom_metric(self) -> int:
           # 实现自定义指标采集
           return value
   ```

2. 在 `monitor_node.py` 中创建对应的话题发布器：

   ```python
   self._custom_pub = self.create_publisher(Int32, 'custom_metric', 10)
   ```

3. 在定时器回调中发布数据。
4. 更新监控面板（Web 前端）显示新指标。

### 添加新的 Web 页面 (fleet_web_ui/frontend)

1. 在 `frontend/src/components/` 下创建新页面目录：

   ```bash
   mkdir frontend/src/components/NewPage
   touch frontend/src/components/NewPage/index.tsx
   touch frontend/src/components/NewPage/NewPage.tsx
   touch frontend/src/components/NewPage/NewPage.module.css
   ```

2. 实现 React 组件：

   ```tsx
   const NewPage: React.FC = () => {
     return <div className={styles.container}>New Page Content</div>;
   };
   export default NewPage;
   ```

3. 在路由配置中添加新页面路径。
4. 在导航菜单中添加入口。
5. 若需要后端数据，先添加对应的 REST API（见下节）。

### 添加新的 REST API (fleet_web_ui/backend)

1. 在 `backend/routes/` 下创建新路由文件（如 `custom.py`）：

   ```python
   from flask import Blueprint, jsonify

   custom_bp = Blueprint('custom', __name__)

   @custom_bp.route('/api/custom', methods=['GET'])
   def get_custom_data():
       # 实现数据获取逻辑
       return jsonify({'data': result})
   ```

2. 在 `app.py` 中注册蓝图：

   ```python
   from routes.custom import custom_bp
   app.register_blueprint(custom_bp)
   ```

3. 若需调用 ROS2 接口，使用 `rclpy` 的客户端封装。

---

## 调试技巧

### ROS2 日志

- **查看节点日志**:

  ```bash
  ros2 log list               # 列出所有日志发布者
  ros2 log show               # 查看日志历史
  ```

- **设置日志级别**:

  ```bash
  ros2 param set /fleet_manager log_level DEBUG
  ros2 param set /fleet_monitor log_level INFO
  ```

- **按模块过滤**:

  ```bash
  ros2 run fleet_manager fleet_manager --ros-args --log-level scheduler:=debug
  ```

### persist_logger

系统内置的持久化日志节点，将 ROS2 日志写入文件：

```bash
# 启动持久化日志（自动保存至 logs/ 目录）
ros2 run fleet_monitor persist_logger

# 指定输出文件
ros2 run fleet_monitor persist_logger --ros-args -p output_file:=/var/log/fleet/system.log
```

日志文件默认按日轮转，详情见 [运维手册 - 日志管理](OPERATIONS.md#日志管理)。

### rviz2 可视化

```bash
# 启动 rviz2 加载预设配置
rviz2 -d src/fleet_management_system/fleet_manager/config/fleet_display.rviz

# 手动添加显示项
# - RobotModel: 显示机器人模型
# - TF: 显示坐标系变换
# - MarkerArray: 显示 waypoint 和 lane 拓扑
# - Path: 显示规划路径
```

### 常用调试命令

```bash
# 查看话题列表
ros2 topic list

# 查看话题数据
ros2 topic echo /robot_01/status

# 查看服务列表
ros2 service list

# 查看节点图
rqt_graph

# 查看参数
ros2 param dump /fleet_manager
```

---

## 单元测试

### C++ 测试 (ament + gtest)

测试文件位于各包的 `test/` 目录下：

```cpp
// fleet_manager/test/test_task_queue.cpp
#include "gtest/gtest.h"
#include "task_queue.hpp"

class TestTaskQueue : public ::testing::Test {
protected:
  void SetUp() override {
    queue_ = std::make_unique<TaskQueue>();
  }

  std::unique_ptr<TaskQueue> queue_;
};

TEST_F(TestTaskQueue, SubmitAndPop) {
  Task task;
  task.task_id = "test_001";
  task.priority = 100;
  queue_->submit(task);

  auto popped = queue_->pop_next();
  ASSERT_TRUE(popped.has_value());
  EXPECT_EQ(popped->task_id, "test_001");
}

TEST_F(TestTaskQueue, PriorityOrder) {
  Task low;
  low.task_id = "low";
  low.priority = 10;

  Task high;
  high.task_id = "high";
  high.priority = 200;

  queue_->submit(low);
  queue_->submit(high);

  auto first = queue_->pop_next();
  EXPECT_EQ(first->task_id, "high");
}
```

在 `CMakeLists.txt` 中添加测试：

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_task_queue test/test_task_queue.cpp)
  target_link_libraries(test_task_queue task_queue_lib)
endif()
```

### Python 测试 (pytest)

```python
# fleet_monitor/test/test_metrics_collector.py
import pytest
from fleet_monitor.metrics_collector import MetricsCollector


class TestMetricsCollector:
    def setup_method(self):
        self.collector = MetricsCollector("test_collector")

    def test_collect_initial_metrics(self):
        metrics = self.collector.collect()
        assert isinstance(metrics, dict)

    def test_collect_after_update(self):
        self.collector._metrics["robots_online"] = 5
        metrics = self.collector.collect()
        assert metrics["robots_online"] == 5

    @pytest.mark.parametrize("key,expected_type", [
        ("robots_online", int),
        ("tasks_pending", int),
    ])
    def test_metric_types(self, key, expected_type):
        metrics = self.collector.collect()
        assert key in metrics
        assert isinstance(metrics[key], expected_type)
```

### 运行测试

```bash
# 运行所有测试
colcon test
colcon test-result --verbose

# 运行特定包的特定测试
colcon test --packages-select fleet_manager --ctest-args -R test_task_queue

# 运行 Python 测试（直接在包目录）
cd src/fleet_management_system/fleet_monitor
python3 -m pytest test/ -v
```

### 测试覆盖率

```bash
# C++ 覆盖率 (gcov)
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-fprofile-arcs -ftest-coverage"
colcon test --packages-select fleet_manager
gcovr --filter src/fleet_manager/src/ --html coverage.html

# Python 覆盖率 (pytest-cov)
python3 -m pytest test/ --cov=fleet_monitor --cov-report=html
```

---

> **文档版本**: v1.0  
> **最后更新**: 2026-05-04
