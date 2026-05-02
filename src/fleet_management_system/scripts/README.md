## scripts 目录说明

这里放的是**仓库级辅助脚本**（诊断、压测、离线仿真、运维工具），不作为 ROS2 可执行节点发布。

### 目录划分

- `diagnostics/`: 在线诊断/规则监控/安全守护
  - `diagnostic_monitor.py`: 综合诊断（轮询 Web API、提交任务、生成诊断日志）
  - `fleet_traffic_rules_monitor.py`: 交通规则约束检查（可 fail-fast）
  - `fleet_violation_auto_cancel_monitor.py`: 检测违规后自动取消任务（压测安全阀）
  - `scheduling_watchdog.py`: 更严格的调度违规监控（含“挂起无活动”等规则）
- `stress/`: 在线 API 压测（依赖 Web 后端 + fleet_manager）
  - `fleet_stress_suite.py`
  - `scheduler_api_stress_test.py`
- `sim/`: 离线仿真/策略可行性验证（不依赖 ROS2）
  - `multi_robot_invariant_sim.py`
  - `multi_robot_route_following_extreme_sim.py`
- `tools/`: 运维/工程工具脚本
  - `sync_maps.sh`: 地图同步到机器人
  - `log_trace.sh`: 日志追踪工具（按 task/robot/chassis/deadlock 输出诊断时间线，支持 `--since 10m` 时间窗口）
- `_lib/`: 脚本公共库（HTTP 调用、交通规则检查等复用逻辑）
  - `cli.py`: argparse 选项复用（base url、轮询参数、robots 过滤）
  - `out.py`: 输出格式辅助（时间戳等）

### 使用约定

- 从仓库根目录运行（例如 `python3 src/fleet_management_system/scripts/diagnostics/diagnostic_monitor.py ...`）。
- `stress/` 与 `diagnostics/` 下的脚本通常默认 Web 后端地址为 `http://localhost:8080`，需要时通过参数或环境变量覆盖。
