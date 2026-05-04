#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
车队管理系统 Web 后端服务器 — 模拟/演示版本

警告：
    此文件仅用于无 ROS2 环境时的 UI 演示与开发调试。
    生产环境请使用 server_ros2.py（集成真实 ROS2 通信）。

功能说明：
    - 提供 REST API 和 WebSocket 接口
    - 数据为硬编码模拟数据，适合前端 UI 开发和功能验证
"""

import asyncio
import json
import logging
from datetime import datetime
from typing import Dict, List, Optional
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel
import uvicorn


# ---- 日志配置 ----

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def log_event(level: str, event: str, **fields):
    """记录结构化日志事件。

    将日志事件和键值对字段拼接为一行字符串输出，方便日志收集系统解析。

    参数:
        level: 日志级别（"info"、"warning"、"error"）
        event: 事件名称标识
        **fields: 附加的键值对字段，字段值为 None 或空字符串时显示为 "-"
    """
    parts = [f"event={event}"]
    for key, value in fields.items():
        parts.append(f"{key}={value if value not in (None, '') else '-'}")
    message = " ".join(parts)
    if level == "error":
        logger.error(message)
    elif level == "warning":
        logger.warning(message)
    else:
        logger.info(message)


# ---- 数据模型 ----

class RobotStatus(BaseModel):
    """机器人状态数据模型。

    属性:
        id: 机器人唯一标识符
        online: 是否在线
        status: 状态枚举（offline / idle / working / charging / error）
        battery: 电池电量百分比
        position: 位置字典（含 x, y, yaw）
        current_task: 当前执行的任务 ID
        speed: 移动速度
        last_update: 最后更新时间
    """
    id: str
    online: bool = False
    status: str = "offline"
    battery: int = 0
    position: Optional[Dict] = None
    current_task: Optional[str] = None
    speed: float = 0.0
    last_update: str = ""


class TaskInfo(BaseModel):
    """任务信息数据模型。

    属性:
        id: 任务唯一标识符
        waypoint_id: 目标航点 ID
        robot_id: 分配的机器人 ID
        status: 状态枚举（pending / assigned / running / completed / failed）
        priority: 优先级（数值越大优先级越高）
        created_at: 创建时间
        completed_at: 完成时间
    """
    id: str
    waypoint_id: str
    robot_id: Optional[str] = None
    status: str = "pending"
    priority: int = 1
    created_at: str = ""
    completed_at: Optional[str] = None


class SubmitTaskRequest(BaseModel):
    """创建任务请求数据模型。

    属性:
        waypoint_id: 目标航点 ID
        priority: 任务优先级
        robot_id: 指定执行的机器人 ID（可选）
    """
    waypoint_id: str
    priority: int = 0
    robot_id: Optional[str] = None


class CommandRequest(BaseModel):
    """控制命令请求数据模型。

    属性:
        command: 命令名称
        payload: 命令附加参数
    """
    command: str
    payload: Dict = {}


# ---- 全局状态 ----

class FleetState:
    """全局状态管理器，维护所有运行时数据。

    管理机器人状态、任务列表、系统日志以及 WebSocket 客户端连接。
    """

    def __init__(self):
        self.robots: Dict[str, RobotStatus] = {}
        self.tasks: Dict[str, TaskInfo] = {}
        self.logs: List[Dict] = []
        self.ros_connected = False
        self.websocket_clients: List[WebSocket] = []

    def add_log(self, level: str, message: str, source: str = "System"):
        """添加一条系统日志。

        日志插入到列表头部，超出 500 条时自动截断尾部。

        参数:
            level: 日志级别（info、warning、error、success 等）
            message: 日志内容
            source: 日志来源标识，默认为 "System"

        返回:
            创建的日志条目字典
        """
        log_entry = {
            "time": datetime.now().strftime("%H:%M:%S"),
            "level": level,
            "message": message,
            "source": source
        }
        self.logs.insert(0, log_entry)
        if len(self.logs) > 500:
            self.logs.pop()
        return log_entry

    async def broadcast(self, message: dict):
        """向所有连接的 WebSocket 客户端广播消息。

        参数:
            message: 要广播的消息字典（将自动序列化为 JSON）
        """
        for client in self.websocket_clients:
            try:
                await client.send_json(message)
            except:
                pass


fleet_state = FleetState()


# ---- FastAPI 应用初始化 ----

@asynccontextmanager
async def lifespan(app: FastAPI):
    """应用生命周期管理。

    启动时初始化模拟数据，关闭时执行清理。
    """
    # ---- 启动时初始化模拟数据 ----
    init_mock_data()
    yield
    # ---- 关闭时清理 ----
    pass


app = FastAPI(
    title="FleetOS API",
    description="车队管理系统 API",
    version="1.0.0",
    lifespan=lifespan
)


# ---- CORS 跨域配置 ----

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ---- 模拟数据初始化 ----

def init_mock_data():
    """初始化硬编码的模拟数据。

    创建 5 台模拟机器人（包含在线和离线状态）和 3 个模拟任务，
    用于在没有真实 ROS2 环境时供前端 UI 开发和调试。
    """
    fleet_state.robots = {
        "AMR-001": RobotStatus(
            id="AMR-001", online=True, status="working", battery=78,
            position={"x": 12.5, "y": 8.3, "yaw": 0.5},
            current_task="T-101", speed=0.5, last_update=datetime.now().strftime("%H:%M:%S")
        ),
        "AMR-002": RobotStatus(
            id="AMR-002", online=True, status="idle", battery=95,
            position={"x": 5.2, "y": 3.1, "yaw": 1.2},
            current_task=None, speed=0, last_update=datetime.now().strftime("%H:%M:%S")
        ),
        "AMR-003": RobotStatus(
            id="AMR-003", online=True, status="charging", battery=45,
            position={"x": 2.0, "y": 10.0, "yaw": 0},
            current_task=None, speed=0, last_update=datetime.now().strftime("%H:%M:%S")
        ),
        "AMR-004": RobotStatus(
            id="AMR-004", online=True, status="working", battery=62,
            position={"x": 18.3, "y": 12.1, "yaw": -0.8},
            current_task="T-102", speed=0.8, last_update=datetime.now().strftime("%H:%M:%S")
        ),
        "AMR-005": RobotStatus(
            id="AMR-005", online=False, status="offline", battery=0,
            position=None, current_task=None, speed=0, last_update="14:20:00"
        ),
    }

    fleet_state.tasks = {
        "T-101": TaskInfo(
            id="T-101", waypoint_id="wp_003", robot_id="AMR-001",
            status="running", priority=3, created_at="14:20:00"
        ),
        "T-102": TaskInfo(
            id="T-102", waypoint_id="wp_005", robot_id="AMR-004",
            status="running", priority=2, created_at="14:25:00"
        ),
        "T-103": TaskInfo(
            id="T-103", waypoint_id="wp_002", robot_id=None,
            status="pending", priority=1, created_at="14:30:00"
        ),
    }

    fleet_state.ros_connected = True
    fleet_state.add_log("info", "系统启动完成")
    fleet_state.add_log("success", "WebSocket 服务就绪")
    log_event("info", "web.mock_init", component="web_backend_mock", robot_count=len(fleet_state.robots), task_count=len(fleet_state.tasks))


# ---- REST API 接口 ----

@app.get("/")
async def root():
    """根路径，返回 API 服务基本信息。"""
    return {"message": "FleetOS API 服务运行中", "version": "1.0.0"}


@app.get("/api/robots")
async def get_robots():
    """获取所有机器人状态列表。"""
    return {"robots": {k: v.model_dump() for k, v in fleet_state.robots.items()}}


@app.get("/api/robots/{robot_id}")
async def get_robot(robot_id: str):
    """获取指定机器人的详细信息。

    参数:
        robot_id: 机器人 ID

    返回:
        机器人状态数据

    异常:
        404: 机器人不存在
    """
    if robot_id not in fleet_state.robots:
        raise HTTPException(status_code=404, detail="机器人不存在")
    return fleet_state.robots[robot_id].model_dump()


@app.get("/api/tasks")
async def get_tasks():
    """获取所有任务列表。"""
    return {"tasks": [v.model_dump() for v in fleet_state.tasks.values()]}


@app.post("/api/tasks")
async def create_task(request: SubmitTaskRequest):
    """创建新任务。

    参数:
        request: 包含航点 ID、优先级和可选机器人 ID 的请求体

    返回:
        创建结果及任务详情
    """
    task_id = f"T-{datetime.now().strftime('%H%M%S')}"
    task = TaskInfo(
        id=task_id,
        waypoint_id=request.waypoint_id,
        robot_id=request.robot_id,
        status="assigned" if request.robot_id else "pending",
        priority=request.priority,
        created_at=datetime.now().strftime("%H:%M:%S")
    )
    fleet_state.tasks[task_id] = task
    log_event(
        "info",
        "task.submit",
        task=task_id,
        robot=request.robot_id or "-",
        state_prev="-",
        state_new=task.status,
        reason="MOCK_API_CREATE",
        detail=f"waypoint:{request.waypoint_id}",
    )

    log = fleet_state.add_log("info", f"创建任务 {task_id} -> {request.waypoint_id}")
    await fleet_state.broadcast({"type": "task_update", "payload": task.model_dump()})

    return {"success": True, "task_id": task_id, "task": task.model_dump()}


@app.delete("/api/tasks/{task_id}")
async def cancel_task(task_id: str):
    """取消指定任务。

    参数:
        task_id: 任务 ID

    返回:
        操作结果

    异常:
        404: 任务不存在
    """
    if task_id not in fleet_state.tasks:
        raise HTTPException(status_code=404, detail="任务不存在")

    fleet_state.tasks[task_id].status = "cancelled"
    log_event("info", "task.cancel", task=task_id, robot=fleet_state.tasks[task_id].robot_id or "-", state_prev="-", state_new="cancelled", reason="MOCK_API_CANCEL")
    fleet_state.add_log("info", f"任务 {task_id} 已取消")

    return {"success": True}


@app.post("/api/command")
async def send_command(request: CommandRequest):
    """发送控制命令。

    参数:
        request: 包含命令名称和负载的请求体

    返回:
        命令执行结果
    """
    log = fleet_state.add_log("info", f"执行命令: {request.command}")
    await fleet_state.broadcast({"type": "log", "level": "info", "message": log["message"]})

    return {"success": True, "command": request.command}


@app.post("/api/emergency_stop")
async def emergency_stop(robot_id: Optional[str] = None):
    """紧急停止指定机器人或全部机器人。

    参数:
        robot_id: 可选，指定机器人 ID。不提供则停止所有机器人。

    返回:
        操作结果
    """
    if robot_id:
        log_event("warning", "robot.emergency_stop", task="-", robot=robot_id, state_prev="-", state_new="stopped", reason="API_EMERGENCY_STOP_SINGLE")
        fleet_state.add_log("warning", f"紧急停止机器人: {robot_id}")
        if robot_id in fleet_state.robots:
            fleet_state.robots[robot_id].status = "idle"
            fleet_state.robots[robot_id].speed = 0
    else:
        log_event("error", "robot.emergency_stop", task="-", robot="*", state_prev="-", state_new="stopped", reason="API_EMERGENCY_STOP_ALL")
        fleet_state.add_log("error", "紧急停止所有机器人")
        for robot in fleet_state.robots.values():
            robot.status = "idle"
            robot.speed = 0

    await fleet_state.broadcast({
        "type": "emergency_stop",
        "robot_id": robot_id
    })

    return {"success": True}


@app.get("/api/status")
async def get_status():
    """获取系统概览状态。

    返回在线机器人数、工作中机器人数、任务统计等概览数据。
    """
    online_count = sum(1 for r in fleet_state.robots.values() if r.online)
    working_count = sum(1 for r in fleet_state.robots.values() if r.status == "working")

    return {
        "ros_connected": fleet_state.ros_connected,
        "total_robots": len(fleet_state.robots),
        "online_robots": online_count,
        "working_robots": working_count,
        "total_tasks": len(fleet_state.tasks),
        "pending_tasks": sum(1 for t in fleet_state.tasks.values() if t.status == "pending"),
        "running_tasks": sum(1 for t in fleet_state.tasks.values() if t.status == "running")
    }


@app.get("/api/logs")
async def get_logs(limit: int = 100):
    """获取系统日志列表。

    参数:
        limit: 返回的最大日志条数，默认 100
    """
    return {"logs": fleet_state.logs[:limit]}


# ---- WebSocket 接口 ----

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket 主端点，处理客户端实时通信。

    支持以下消息类型：
        - ping: 心跳检测，回复 pong
        - get_fleet_status: 获取当前车队状态
        - submit_task: 提交新任务
        - cancel_task: 取消任务
        - emergency_stop: 紧急停止
        - recall_robot: 召回机器人

    连接建立后自动发送初始状态（含机器人列表、任务列表、ROS2 连接状态）。
    """
    await websocket.accept()
    fleet_state.websocket_clients.append(websocket)
    fleet_state.add_log("info", "WebSocket 客户端连接")

    try:
        # ---- 发送初始状态 ----
        await websocket.send_json({
            "type": "init",
            "payload": {
                "robots": {k: v.model_dump() for k, v in fleet_state.robots.items()},
                "tasks": [v.model_dump() for v in fleet_state.tasks.values()],
                "ros_connected": fleet_state.ros_connected
            }
        })

        while True:
            data = await websocket.receive_json()

            # ---- 消息路由 ----
            msg_type = data.get("type")
            payload = data.get("payload", {})

            if msg_type == "ping":
                await websocket.send_json({"type": "pong"})

            elif msg_type == "get_fleet_status":
                await websocket.send_json({
                    "type": "fleet_status",
                    "payload": {k: v.model_dump() for k, v in fleet_state.robots.items()}
                })

            elif msg_type == "submit_task":
                task_id = f"T-{datetime.now().strftime('%H%M%S')}"
                task = TaskInfo(
                    id=task_id,
                    waypoint_id=payload.get("waypoint_id"),
                    robot_id=payload.get("robot_id"),
                    status="assigned" if payload.get("robot_id") else "pending",
                    priority=payload.get("priority", 0),
                    created_at=datetime.now().strftime("%H:%M:%S")
                )
                fleet_state.tasks[task_id] = task
                fleet_state.add_log("info", f"通过WebSocket创建任务 {task_id}")
                await websocket.send_json({"type": "task_created", "task": task.model_dump()})

            elif msg_type == "cancel_task":
                task_id = payload.get("task_id")
                if task_id in fleet_state.tasks:
                    fleet_state.tasks[task_id].status = "cancelled"
                    fleet_state.add_log("info", f"任务 {task_id} 已取消")

            elif msg_type == "emergency_stop":
                robot_id = payload.get("robot_id")
                if robot_id and robot_id in fleet_state.robots:
                    fleet_state.robots[robot_id].status = "idle"
                    fleet_state.robots[robot_id].speed = 0
                    fleet_state.add_log("warning", f"机器人 {robot_id} 紧急停止")
                await websocket.send_json({"type": "stop_confirmed", "robot_id": robot_id})

            elif msg_type == "recall_robot":
                robot_id = payload.get("robot_id")
                if robot_id and robot_id in fleet_state.robots:
                    fleet_state.robots[robot_id].current_task = None
                    fleet_state.robots[robot_id].status = "idle"
                    fleet_state.add_log("info", f"机器人 {robot_id} 已召回")

    except WebSocketDisconnect:
        fleet_state.websocket_clients.remove(websocket)
        fleet_state.add_log("info", "WebSocket 客户端断开")
    except Exception as e:
        log_event("error", "websocket.error", detail=str(e))
        if websocket in fleet_state.websocket_clients:
            fleet_state.websocket_clients.remove(websocket)


# ---- 程序入口 ----

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
