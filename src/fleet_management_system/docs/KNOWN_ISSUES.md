# 已知问题

本文件用于记录当前仓库中已经确认存在、但尚未在代码层彻底修复的问题。

## 待解决

### 1. CORS 配置使用通配符 `allow_origins=["*"]`
- **位置**: `apps/fleet_web_ui/backend/server_ros2.py`
- **影响**: 生产环境中可能导致跨域安全风险
- **规避**: 部署前修改为具体前端域名
- **优先级**: 中

### 2. Zenoh 桥接配置含硬编码 IP 地址
- **位置**: `src/fleet_management_system/config/zenoh/host-bridge.json5`
- **影响**: 换网需手动修改配置
- **规避**: 部署时替换为实际路由地址
- **优先级**: 低

### 3. `fleet_manager_node.cpp` 文件过大（3000+ 行）
- **位置**: `src/fleet_manager/src/fleet_manager_node.cpp`
- **影响**: 可维护性差，但功能正常
- **规避**: 按模块拆分（建议未来重构）
- **优先级**: 低