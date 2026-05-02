# 文档总览

本文档集是当前仓库的唯一维护版本。所有运行说明、部署说明和模块说明都收口在 docs 目录，避免根目录 README、包内 README 和配置目录 README 长期分叉。

## 建议阅读顺序

1. 先读 ARCHITECTURE.md，建立对真实包结构和运行链的整体认知。
2. 再读 BUILD_AND_RUN.md，按当前可执行的方式完成构建和启动。
3. 根据工作内容继续阅读 WEB_UI.md、TRAFFIC_EDITOR.md、ZENOH_DEPLOYMENT.md 和 MAP_DISTRIBUTION.md。
4. 遇到不一致或明显异常时，优先检查 KNOWN_ISSUES.md。

## 文档目录

- ARCHITECTURE.md
  说明源码结构、包职责、接口层和运行链路。
- BUILD_AND_RUN.md
  说明依赖、编译、手工启动顺序、验证命令和当前推荐工作流。
- WEB_UI.md
  说明 Vue 前端、FastAPI 后端、端口、API 和 WebSocket 行为。
- TRAFFIC_EDITOR.md
  说明交通图编辑器依赖、启动方式、编辑流程和文件格式。
- ZENOH_DEPLOYMENT.md
  说明主机端 Router、Bridge 和机器人端 Bridge 的部署方式。
- MAP_DISTRIBUTION.md
  说明地图文件组成、地图同步策略和发布建议。
- KNOWN_ISSUES.md
  记录当前代码与脚本层面已确认但尚未修复的问题。

## 文档范围

本套文档只描述当前仓库里已经存在并能被代码证实的能力，不再保留历史上未实现或已经失效的功能说明。

尤其需要注意：

- Web UI 目录 `apps/fleet_web_ui` 不是 colcon 包，而是独立的 Vue + FastAPI 工程。
- 一部分脚本和配置仍存在绝对路径或命名误导，详见 KNOWN_ISSUES.md。