#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
车队管理系统 Web 后端服务器
集成 ROS2 - 仅支持真实底盘

环境变量:
  FLEET_STATUS_TOPIC — 车队状态话题，默认 /fleet_manager/fleet_status_traffic
  （含交通定位字段）。若无 fleet_manager，可设为 /fleet_monitor/fleet_status。
"""

import asyncio
import json
import logging
import math
import threading
import yaml
import os
import base64
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
import uvicorn

# ROS2 导入
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from fleet_msgs.msg import RobotStatus, FleetStatus, TaskInfo as TaskInfoMsg
    from fleet_msgs.srv import SubmitTask, CancelTask
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("警告: 未找到ROS2，后端将启动但无法连接底盘")

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

TRAFFIC_MAP_FILE = os.environ.get("FLEET_TRAFFIC_MAP_FILE")


def log_event(level: str, event: str, **fields):
    parts = [f"event={event}"]
    for key, value in fields.items():
        parts.append(f"{key}={value if value not in (None, '') else '-'}")
    msg = " ".join(parts)
    if level == "error":
        logger.error(msg)
    elif level == "warning":
        logger.warning(msg)
    else:
        logger.info(msg)

# ==================== Settings persistence ====================

SETTINGS_FILE = Path(os.environ.get(
    "FLEET_WEB_SETTINGS_FILE",
    str(Path(__file__).resolve().parents[3] / "src" / "fleet_management_system" / "config" / "webui_settings.json")
))


def load_settings() -> dict:
    try:
        if SETTINGS_FILE.exists():
            return json.loads(SETTINGS_FILE.read_text(encoding="utf-8"))
    except Exception as e:
        logger.warning(f"读取 settings 失败: {e}")
    return {}


def save_settings(data: dict) -> None:
    SETTINGS_FILE.parent.mkdir(parents=True, exist_ok=True)
    SETTINGS_FILE.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


class SettingsModel(BaseModel):
    ros_domain_id: Optional[int] = None
    zenoh_router: Optional[str] = None
    ws_port: Optional[int] = None

# 数据模型
class RobotStatusModel(BaseModel):
    id: str
    online: bool = False
    status: str = "offline"
    battery: int = 0
    position: Optional[Dict] = None
    current_task: Optional[str] = None
    speed: float = 0.0
    last_update: str = ""

class TaskInfo(BaseModel):
    id: str
    waypoint_id: str
    robot_id: Optional[str] = None
    status: str = "pending"
    priority: int = 1
    task_type: int = 1        # 1=CRUISE, 2=LOAD, 3=UNLOAD, 4=SITE_SPECIFIC
    site_code: int = 0        # 业务代码
    created_at: str = ""

class SubmitTaskRequest(BaseModel):
    waypoint_id: str
    priority: int = 0
    robot_id: Optional[str] = None
    task_type: int = 1        # 1=CRUISE, 2=LOAD, 3=UNLOAD, 4=SITE_SPECIFIC
    site_code: int = 0        # 业务代码

class AssignTaskRequest(BaseModel):
    robot_id: str

class CommandRequest(BaseModel):
    command: str
    payload: Dict = {}


# ROS2 节点
class ROSBridgeNode(Node if HAS_ROS else object):
    """ROS2 桥接节点"""
    
    def __init__(self, state_manager):
        if HAS_ROS:
            super().__init__('fleet_web_bridge')
        self.state = state_manager
        
        if HAS_ROS:
            # 订阅车队状态：默认用 fleet_manager 融合话题（含交通定位），否则 monitor 无航点/线段信息
            fleet_topic = os.environ.get(
                'FLEET_STATUS_TOPIC', '/fleet_manager/fleet_status_traffic')
            self.fleet_sub = self.create_subscription(
                FleetStatus, fleet_topic, self.fleet_callback, 10)
            self.get_logger().info(f'订阅车队状态话题: {fleet_topic}')
            
            # 订阅任务状态更新
            self.task_status_sub = self.create_subscription(
                TaskInfoMsg, '/fleet_manager/task_status', self.task_status_callback, 10
            )
            
            # 任务服务客户端
            self.task_client = self.create_client(SubmitTask, '/fleet_manager/submit_task')

            # 取消任务服务客户端
            self.cancel_task_client = self.create_client(CancelTask, '/fleet_manager/cancel_task')

            # 移除机器人服务客户端
            try:
                from fleet_msgs.srv import RemoveRobot
                self.remove_robot_client = self.create_client(RemoveRobot, '/fleet_manager/remove_robot')
            except ImportError:
                self.remove_robot_client = None
            
            self.get_logger().info('ROS2 桥接节点启动')
    
    def task_status_callback(self, msg):
        """任务状态回调"""
        task_id = msg.task_id
        status = msg.status
        
        self.get_logger().info(
            f"event=task.status_update source=web_bridge task={task_id} state_new={status}"
        )
        
        # 更新本地任务状态
        if task_id in self.state.tasks:
            self.state.tasks[task_id].status = status
            self.state.tasks[task_id].robot_id = msg.assigned_robot_id if msg.assigned_robot_id else self.state.tasks[task_id].robot_id
        else:
            # 新任务，添加到列表
            self.state.tasks[task_id] = TaskInfo(
                id=task_id,
                waypoint_id=msg.waypoint_id,
                robot_id=msg.assigned_robot_id if msg.assigned_robot_id else None,
                status=status,
                priority=msg.priority,
                task_type=msg.task_type if hasattr(msg, 'task_type') else 1,
                site_code=msg.site_code if hasattr(msg, 'site_code') else 0,
                created_at=datetime.now().strftime("%H:%M:%S")
            )
        
        # 广播任务更新
        task_dict = self.state.tasks[task_id].dict() if hasattr(self.state.tasks[task_id], 'dict') else dict(self.state.tasks[task_id])
        asyncio.run_coroutine_threadsafe(
            self.state.broadcast({'type': 'task_update', 'payload': task_dict}),
            self.state.event_loop
        )
        
        # 添加日志
        if status == "completed":
            self.state.add_log("info", f"任务 {task_id} 已完成")
        elif status == "failed":
            self.state.add_log("error", f"任务 {task_id} 失败")
        elif status == "assigned":
            self.state.add_log("info", f"任务 {task_id} 已分配给 {msg.assigned_robot_id}")
        elif status == "executing":
            self.state.add_log("info", f"任务 {task_id} 底盘执行中")
    
    def fleet_callback(self, msg: FleetStatus):
        """车队状态回调"""
        new_robots = {}
        for robot_msg in msg.robots:
            # 跳过已移除出队的机器人（除非它重新上线）
            if robot_msg.robot_id in self.state.removed_robots:
                if robot_msg.connection_status == "online":
                    # 重新上线，自动解除黑名单
                    self.state.removed_robots.discard(robot_msg.robot_id)
                else:
                    continue  # 仍然离线，继续忽略
            # 世界坐标转像素坐标
            world_x = robot_msg.current_pose.position.x
            world_y = robot_msg.current_pose.position.y
            pixel_x, pixel_y = self.state.world_to_pixel(world_x, world_y)
            
            is_online = robot_msg.connection_status == "online"
            
            # offline 机器人：status 强制为 offline，保留最后已知位姿
            if is_online:
                nav_status = robot_msg.nav_status if robot_msg.nav_status else 'unknown'
            else:
                nav_status = 'offline'
            
            new_robots[robot_msg.robot_id] = {
                'id': robot_msg.robot_id,
                'online': is_online,
                'status': nav_status,
                'battery': robot_msg.battery_percentage,
                'position': {
                    'x': pixel_x,  # 使用像素坐标
                    'y': pixel_y,
                    'yaw': 2 * math.atan2(
                        robot_msg.current_pose.orientation.z,
                        robot_msg.current_pose.orientation.w
                    ),
                    # 保留世界坐标用于调试
                    'world_x': world_x,
                    'world_y': world_y
                },
                'current_task': robot_msg.current_task_id if robot_msg.current_task_id else None,
                'nav_status': robot_msg.nav_status if robot_msg.nav_status else '',
                'connection_status': robot_msg.connection_status if hasattr(robot_msg, 'connection_status') else 'unknown',
                'location_type': robot_msg.location_type if hasattr(robot_msg, 'location_type') else 'unknown',
                'current_waypoint': robot_msg.current_waypoint if hasattr(robot_msg, 'current_waypoint') else '',
                'current_segment': robot_msg.current_segment if hasattr(robot_msg, 'current_segment') else '',
                'planned_route': list(robot_msg.planned_route) if hasattr(robot_msg, 'planned_route') else [],
                'last_update': datetime.now().strftime("%H:%M:%S")
            }
        
        # 合并而非覆盖：保留不在新消息中的旧机器人（标记为 offline）
        # 这样掉线的机器人不会从列表消失
        for robot_id, old_data in list(self.state.robots.items()):
            if robot_id in self.state.removed_robots:
                continue  # 已移除出队的机器人不保留
            if robot_id not in new_robots:
                # 保留旧数据但标记为 offline
                offline_data = dict(old_data)
                offline_data['online'] = False
                offline_data['status'] = 'offline'
                offline_data['connection_status'] = 'offline'
                offline_data['nav_status'] = 'offline'
                offline_data['planned_route'] = []
                new_robots[robot_id] = offline_data
        
        self.state.robots = new_robots
        # 触发广播
        asyncio.run_coroutine_threadsafe(
            self.state.broadcast({'type': 'fleet_status', 'payload': new_robots}),
            self.state.event_loop
        )


# 全局状态管理器
class FleetState:
    def __init__(self):
        self.robots: Dict[str, dict] = {}
        self.removed_robots: set = set()  # 移除出队的黑名单，不再接受其状态更新
        self.tasks: Dict[str, TaskInfo] = {}
        self.waypoints: List[Dict] = []
        self.map_data: Dict = {}  # 地图数据
        self.map_dir: str = ""    # 地图目录
        self.map_resolution: float = 0.05  # 地图分辨率
        self.map_origin: List[float] = [0.0, 0.0, 0.0]  # 地图原点
        self.map_height: int = 0  # 地图图像高度（像素）
        self.logs: List[Dict] = []
        self.ros_connected = False
        self.websocket_clients: List[WebSocket] = []
        self.ros_node: Optional[ROSBridgeNode] = None
        self.event_loop = None
    
    def world_to_pixel(self, world_x: float, world_y: float) -> tuple:
        """世界坐标转像素坐标
        
        ROS地图origin是图像左下角在世界坐标系中的位置。
        图像坐标系Y轴向下，世界坐标系Y轴向上，需要翻转。
        
        转换公式：
        pixel_x = (world_x - origin_x) / resolution
        pixel_y = map_height - (world_y - origin_y) / resolution
        """
        if self.map_height == 0:
            logger.warning("map_height 为 0，坐标转换可能不正确！")
            
        pixel_x = (world_x - self.map_origin[0]) / self.map_resolution
        # Y轴翻转：图像Y轴向下，世界Y轴向上
        pixel_y = self.map_height - (world_y - self.map_origin[1]) / self.map_resolution
        return pixel_x, pixel_y
    
    def add_log(self, level: str, message: str, source: str = "System"):
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
        for client in self.websocket_clients[:]:
            try:
                await client.send_json(message)
            except:
                self.websocket_clients.remove(client)

fleet_state = FleetState()


def load_traffic_map(map_path: str = None):
    """加载交通地图（包含航点和地图图片信息）"""
    if map_path is None:
        # 优先使用仓库内相对路径，避免写死用户目录
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../"))
        possible_paths = [
            os.path.join(repo_root, "maps", "map0", "rmf_map0.yaml"),
            os.path.join(os.path.dirname(__file__), "../../../maps/map0/rmf_map0.yaml"),
        ]
        for p in possible_paths:
            if os.path.exists(p):
                map_path = p
                break
    
    if not map_path or not os.path.exists(map_path):
        log_event("warning", "map.load_skip", reason="MAP_FILE_NOT_FOUND", map_path=map_path or "-")
        return
    
    try:
        with open(map_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        
        # 保存地图目录
        fleet_state.map_dir = os.path.dirname(os.path.abspath(map_path))
        fleet_state.map_data = data
        
        # 加载地图元数据（分辨率和原点）
        if 'map_resolution' in data:
            fleet_state.map_resolution = data['map_resolution']
        if 'map_origin' in data:
            fleet_state.map_origin = data['map_origin']
        
        # 获取地图图像高度（用于Y轴翻转）
        map_image = data.get('map_image', '')
        if map_image:
            image_path = os.path.join(fleet_state.map_dir, map_image) if not os.path.isabs(map_image) else map_image
            if os.path.exists(image_path):
                # 尝试用 PIL 读取
                try:
                    from PIL import Image
                    with Image.open(image_path) as img:
                        fleet_state.map_height = img.height
                        log_event("info", "map.image_meta", reason="PIL_HEIGHT_READ", map_height=fleet_state.map_height)
                except Exception as e:
                    log_event("warning", "map.image_meta", reason="PIL_HEIGHT_READ_FAILED", detail=str(e))
                    # 备选方案：直接读取 PGM 文件头
                    try:
                        with open(image_path, 'rb') as f:
                            magic = f.readline().decode('ascii').strip()
                            if magic in ('P5', 'P2'):
                                # 跳过注释行
                                line = f.readline().decode('ascii')
                                while line.startswith('#'):
                                    line = f.readline().decode('ascii')
                                # 读取宽度和高度
                                dims = line.strip().split()
                                if len(dims) >= 2:
                                    fleet_state.map_height = int(dims[1])
                                    log_event("info", "map.image_meta", reason="PGM_HEADER_HEIGHT_READ", map_height=fleet_state.map_height)
                    except Exception as e2:
                        log_event("error", "map.image_meta", reason="PGM_HEADER_READ_FAILED", detail=str(e2))
            else:
                log_event("warning", "map.image_meta", reason="MAP_IMAGE_NOT_FOUND", image_path=image_path)
        else:
            log_event("warning", "map.image_meta", reason="MAP_IMAGE_FIELD_MISSING")
        
        log_event(
            "info",
            "map.loaded",
            resolution=fleet_state.map_resolution,
            origin=fleet_state.map_origin,
            map_height=fleet_state.map_height,
        )
        
        # 加载航点
        waypoints = []
        for wp in data.get('waypoints', []):
            waypoints.append({
                'id': wp['id'],
                'name': wp.get('name', wp['id']),
                'x': wp['position']['x'],
                'y': wp['position']['y'],
                'is_charging_station': wp.get('is_charging_station', False),
                'is_parking_spot': wp.get('is_parking_spot', False),
                'connections': wp.get('connections', [])
            })
        
        fleet_state.waypoints = waypoints
        log_event("info", "map.loaded_summary", map_path=map_path, waypoint_count=len(waypoints), map_dir=fleet_state.map_dir)
        
        # 检查地图图片
        map_yaml_path = data.get('map_yaml_path', '')
        map_image = data.get('map_image', '')
        if map_yaml_path:
            log_event("info", "map.loaded_asset", asset_type="map_yaml", path=map_yaml_path)
        if map_image:
            log_event("info", "map.loaded_asset", asset_type="map_image", path=map_image)
        
    except Exception as e:
        log_event("error", "map.load_failed", detail=str(e))


# FastAPI 生命周期
@asynccontextmanager
async def lifespan(app: FastAPI):
    fleet_state.event_loop = asyncio.get_event_loop()

    # 先加载 settings（用于 ROS_DOMAIN_ID 等必须在 rclpy.init 前设置的环境变量）
    s = load_settings()
    ros_domain_id = s.get("ros_domain_id", None)
    if ros_domain_id is not None and "ROS_DOMAIN_ID" not in os.environ:
        os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
        logger.info(f"从 settings 应用 ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}")
    
    # 加载交通地图
    if not fleet_state.map_data:
        load_traffic_map(TRAFFIC_MAP_FILE)
    
    # 初始化 ROS2
    if HAS_ROS:
        try:
            rclpy.init()
            fleet_state.ros_node = ROSBridgeNode(fleet_state)
            fleet_state.ros_connected = True
            fleet_state.add_log("success", "ROS2 连接成功，等待底盘上线...")
            
            def spin_node():
                executor = MultiThreadedExecutor()
                executor.add_node(fleet_state.ros_node)
                executor.spin()
            
            spin_thread = threading.Thread(target=spin_node, daemon=True)
            spin_thread.start()
            
        except Exception as e:
            logger.error(f"ROS2初始化失败: {e}")
            fleet_state.add_log("error", f"ROS2初始化失败: {e}")
            fleet_state.ros_connected = False
    else:
        fleet_state.add_log("warning", "ROS2未安装，无法连接底盘")
        fleet_state.ros_connected = False
    
    yield
    
    # 清理
    if HAS_ROS and fleet_state.ros_node:
        fleet_state.ros_node.destroy_node()
        rclpy.shutdown()


app = FastAPI(
    title="FleetOS API",
    description="车队管理系统 API (ROS2集成版)",
    version="1.0.0",
    lifespan=lifespan
)

# CORS 配置
# TODO: 生产环境应将 allow_origins 限制为具体域名，而非 "*"
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ==================== REST API ====================

@app.get("/")
async def root():
    return {
        "message": "FleetOS API 服务运行中",
        "version": "1.0.0",
        "ros_connected": fleet_state.ros_connected,
        "robots_online": len([r for r in fleet_state.robots.values() if r.get('online')]),
        "waypoints_count": len(fleet_state.waypoints)
    }

@app.get("/api/robots")
async def get_robots():
    """获取所有机器人状态"""
    return {"robots": fleet_state.robots}

@app.get("/api/robots/{robot_id}")
async def get_robot(robot_id: str):
    """获取单个机器人状态"""
    if robot_id not in fleet_state.robots:
        raise HTTPException(status_code=404, detail="机器人不存在")
    return fleet_state.robots[robot_id]

@app.post("/api/robots/{robot_id}/recall")
async def recall_robot(robot_id: str):
    """召回机器人：当前实现为取消该机器人正在执行的任务（可靠停下/释放调度）。"""
    if robot_id not in fleet_state.robots:
        raise HTTPException(status_code=404, detail="机器人不存在")

    current_task = (fleet_state.robots.get(robot_id) or {}).get("current_task")
    if not current_task:
        return {"success": True, "message": "机器人当前无任务"}

    # 复用取消任务逻辑（会尝试调用 ROS2 侧取消服务）
    result = await cancel_task(current_task)
    fleet_state.add_log("info", f"召回 {robot_id}: cancel {current_task}")
    return {"success": True, "robot_id": robot_id, "task_id": current_task, "cancel": result}


@app.post("/api/robots/{robot_id}/remove")
async def remove_robot(robot_id: str):
    """移除机器人出队：释放所有航道/航线锁，取消任务，清理状态。用于离线机器人检修。"""
    if robot_id not in fleet_state.robots:
        raise HTTPException(status_code=404, detail="机器人不存在")

    # 调用 ROS2 侧的 RemoveRobot 服务
    if fleet_state.ros_node and getattr(fleet_state.ros_node, "remove_robot_client", None) is not None:
        client = fleet_state.ros_node.remove_robot_client
        if client.service_is_ready():
            from fleet_msgs.srv import RemoveRobot
            ros_request = RemoveRobot.Request()
            ros_request.robot_id = robot_id
            try:
                import time
                future = client.call_async(ros_request)
                start_time = time.time()
                while not future.done():
                    if time.time() - start_time > 10.0:
                        raise HTTPException(status_code=504, detail="移除机器人超时，ROS2服务响应时间过长")
                    await asyncio.sleep(0.1)
                response = future.result()
                if response and getattr(response, "success", False):
                    # 从本地状态中移除该机器人
                    if robot_id in fleet_state.robots:
                        del fleet_state.robots[robot_id]
                    # 加入黑名单，防止 fleet_callback 将其重新加入
                    fleet_state.removed_robots.add(robot_id)
                    fleet_state.add_log("warn", f"机器人 {robot_id} 已从车队移除，所有锁已释放")
                    await fleet_state.broadcast({
                        "type": "robot_removed",
                        "payload": {"robot_id": robot_id}
                    })
                    return {"success": True, "message": response.message or "机器人已从车队移除"}
                else:
                    msg = getattr(response, "message", "未知错误") if response else "无响应"
                    raise HTTPException(status_code=500, detail=f"移除机器人失败: {msg}")
            except HTTPException:
                raise
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"ROS2服务调用异常: {str(e)}")
        else:
            raise HTTPException(status_code=503, detail="ROS2 移除机器人服务未就绪")
    else:
        raise HTTPException(status_code=503, detail="ROS2 未连接或移除机器人服务不可用")


@app.get("/api/settings")
async def get_settings():
    """获取 WebUI settings（持久化在服务端文件）"""
    return {"settings": load_settings(), "settings_file": str(SETTINGS_FILE)}


@app.post("/api/settings")
async def update_settings(req: SettingsModel):
    """更新 WebUI settings，并提示需要重启的项"""
    data = load_settings()
    incoming = req.model_dump(exclude_none=True)
    data.update(incoming)
    save_settings(data)

    restart_required = []
    if "ros_domain_id" in incoming:
        restart_required.append("ros_domain_id")
    if "zenoh_router" in incoming:
        restart_required.append("zenoh_router")

    return {"success": True, "settings": data, "restart_required": restart_required}

@app.get("/api/map/waypoints")
async def get_waypoints():
    """获取地图航点列表"""
    return {"waypoints": fleet_state.waypoints}

@app.get("/api/map/info")
async def get_map_info():
    """获取地图信息"""
    return {
        "map_data": fleet_state.map_data,
        "map_dir": fleet_state.map_dir,
        "waypoints_count": len(fleet_state.waypoints)
    }

@app.get("/api/map/image")
async def get_map_image():
    """获取地图图片（返回base64编码，PGM自动转PNG）"""
    map_data = fleet_state.map_data
    map_dir = fleet_state.map_dir
    
    # 优先使用 map_image 字段
    map_image = map_data.get('map_image', '')
    
    if not map_image:
        # 如果没有 map_image，尝试从 map_yaml_path 加载
        map_yaml_path = map_data.get('map_yaml_path', '')
        if map_yaml_path:
            # 加载 map yaml 获取图片名
            yaml_full_path = os.path.join(map_dir, map_yaml_path) if not os.path.isabs(map_yaml_path) else map_yaml_path
            if os.path.exists(yaml_full_path):
                try:
                    with open(yaml_full_path, 'r') as f:
                        map_yaml = yaml.safe_load(f)
                    map_image = map_yaml.get('image', '')
                    map_dir = os.path.dirname(yaml_full_path)
                except:
                    pass
        1
    if not map_image:
        raise HTTPException(status_code=404, detail="地图图片未配置")
    
    # 构建图片完整路径
    if os.path.isabs(map_image):
        image_path = map_image
    else:
        image_path = os.path.join(map_dir, map_image)
    
    if not os.path.exists(image_path):
        raise HTTPException(status_code=404, detail=f"地图图片文件不存在: {map_image}")
    
    # 读取图片并转为 PNG 格式的 base64
    try:
        ext = os.path.splitext(map_image)[1].lower()
        
        # PGM 格式需要转换为 PNG（浏览器不支持 PGM）
        if ext == '.pgm':
            from PIL import Image
            from io import BytesIO
            
            # 读取 PGM 并转换为 PNG
            img = Image.open(image_path)
            buffer = BytesIO()
            img.save(buffer, format='PNG')
            image_data = buffer.getvalue()
            mime_type = 'image/png'
            logger.info(f"PGM 图片已转换为 PNG: {map_image}")
        else:
            with open(image_path, 'rb') as f:
                image_data = f.read()
            mime_types = {
                '.png': 'image/png',
                '.jpg': 'image/jpeg',
                '.jpeg': 'image/jpeg',
                '.bmp': 'image/bmp'
            }
            mime_type = mime_types.get(ext, 'application/octet-stream')
        
        base64_data = base64.b64encode(image_data).decode('utf-8')
        
        return {
            "image": f"data:{mime_type};base64,{base64_data}",
            "filename": map_image,
            "size": len(image_data)
        }
    except Exception as e:
        log_event("error", "map.image_read_failed", detail=str(e))
        raise HTTPException(status_code=500, detail=f"读取地图图片失败: {str(e)}")

@app.get("/api/map/image/file")
async def get_map_image_file():
    """获取地图图片文件（直接返回文件）"""
    map_data = fleet_state.map_data
    map_dir = fleet_state.map_dir
    
    map_image = map_data.get('map_image', '')
    
    if not map_image:
        map_yaml_path = map_data.get('map_yaml_path', '')
        if map_yaml_path:
            yaml_full_path = os.path.join(map_dir, map_yaml_path) if not os.path.isabs(map_yaml_path) else map_yaml_path
            if os.path.exists(yaml_full_path):
                try:
                    with open(yaml_full_path, 'r') as f:
                        map_yaml = yaml.safe_load(f)
                    map_image = map_yaml.get('image', '')
                    map_dir = os.path.dirname(yaml_full_path)
                except:
                    pass
    
    if not map_image:
        raise HTTPException(status_code=404, detail="地图图片未配置")
    
    if os.path.isabs(map_image):
        image_path = map_image
    else:
        image_path = os.path.join(map_dir, map_image)
    
    if not os.path.exists(image_path):
        raise HTTPException(status_code=404, detail=f"地图图片文件不存在")
    
    return FileResponse(image_path)

@app.get("/api/tasks")
async def get_tasks():
    """获取所有任务"""
    return {"tasks": [v.dict() if hasattr(v, 'dict') else dict(v) for v in fleet_state.tasks.values()]}

@app.post("/api/tasks")
async def create_task(request: SubmitTaskRequest):
    """创建新任务"""
    if not fleet_state.ros_connected:
        raise HTTPException(status_code=503, detail="ROS2未连接，无法创建任务")
    
    # 让 fleet_manager 侧做“空闲底盘选择”
    # 如果 Web 没有指定 robot_id，就把 robot_id 置空，触发 fleet_manager 的自动调度；
    # 这样避免 Web 因 nav_status 采样延迟误判繁忙车为空闲而把任务固定到错误底盘。
    robot_id = request.robot_id or ""
    if robot_id:
        if robot_id not in fleet_state.robots:
            raise HTTPException(status_code=404, detail=f"机器人 {robot_id} 不存在或未上线")
    
    task_id = ""
    
    if fleet_state.ros_node and fleet_state.ros_node.task_client.service_is_ready():
        ros_request = SubmitTask.Request()
        ros_request.robot_id = robot_id
        ros_request.waypoint_id = request.waypoint_id
        ros_request.priority = request.priority
        ros_request.task_type = request.task_type
        ros_request.site_code = request.site_code
        
        try:
            import time
            future = fleet_state.ros_node.task_client.call_async(ros_request)
            
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > 10.0:
                    raise HTTPException(status_code=504, detail="任务提交超时，ROS2服务响应时间过长")
                await asyncio.sleep(0.1)
            
            response = future.result()
            if not response.success:
                raise HTTPException(status_code=400, detail=response.message)
            task_id = response.task_id if response.task_id else f"T-{datetime.now().strftime('%H%M%S')}"
        except Exception as e:
            if isinstance(e, HTTPException):
                raise e
            log_event("error", "task.submit_failed", reason="ROS2_SUBMIT_EXCEPTION", detail=str(e))
            raise HTTPException(status_code=500, detail=f"ROS2任务提交失败: {str(e)}")
    else:
        raise HTTPException(status_code=503, detail="ROS2任务服务不可用，请检查fleet_manager节点是否运行")
    
    task = TaskInfo(
        id=task_id,
        waypoint_id=request.waypoint_id,
        robot_id=robot_id if robot_id else None,
        status="pending",
        priority=request.priority,
        task_type=request.task_type,
        site_code=request.site_code,
        # 用 ISO 时间，便于前端排序/展示
        created_at=datetime.now().isoformat(timespec="seconds")
    )
    fleet_state.tasks[task_id] = task
    
    fleet_state.add_log("info", f"创建任务 {task_id}: {robot_id} -> {request.waypoint_id}")
    await fleet_state.broadcast({"type": "task_update", "payload": task.dict() if hasattr(task, 'dict') else dict(task)})
    
    return {"success": True, "task_id": task_id, "task": task.dict() if hasattr(task, 'dict') else dict(task)}

@app.patch("/api/tasks/{task_id}")
async def assign_task(task_id: str, request: AssignTaskRequest):
    """手动分配待分配任务到指定机器人"""
    if task_id not in fleet_state.tasks:
        raise HTTPException(status_code=404, detail="任务不存在")

    if request.robot_id not in fleet_state.robots:
        raise HTTPException(status_code=404, detail=f"机器人 {request.robot_id} 不存在或未上线")

    task = fleet_state.tasks[task_id]
    if task.status not in ("pending", "assigned"):
        raise HTTPException(status_code=400, detail=f"任务状态为 {task.status}，不可重新分配")

    task.robot_id = request.robot_id
    task.status = "assigned"
    fleet_state.tasks[task_id] = task

    task_dict = task.dict() if hasattr(task, "dict") else dict(task)
    await fleet_state.broadcast({"type": "task_update", "payload": task_dict})
    fleet_state.add_log("info", f"任务 {task_id} 手动分配给 {request.robot_id}")
    return {"success": True, "task": task_dict}

@app.delete("/api/tasks/{task_id}")
async def cancel_task(task_id: str):
    """取消任务"""
    if task_id not in fleet_state.tasks:
        raise HTTPException(status_code=404, detail="任务不存在")

    # 尝试调用 ROS2 侧的取消服务（真正取消正在执行的 Nav2 goal）
    if fleet_state.ros_node and getattr(fleet_state.ros_node, "cancel_task_client", None) is not None:
        client = fleet_state.ros_node.cancel_task_client
        if client.service_is_ready():
            from fleet_msgs.srv import CancelTask
            ros_request = CancelTask.Request()
            ros_request.task_id = task_id
            try:
                import time
                future = client.call_async(ros_request)
                start_time = time.time()
                while not future.done():
                    if time.time() - start_time > 10.0:
                        raise HTTPException(status_code=504, detail="任务取消超时，ROS2服务响应时间过长")
                    await asyncio.sleep(0.1)
                response = future.result()
                if response and getattr(response, "success", False):
                    fleet_state.tasks[task_id].status = "cancelled"
                    fleet_state.add_log("info", f"任务 {task_id} 已取消")
                    return {"success": True}
                else:
                    detail = getattr(response, "message", "") if response else "ROS2取消失败"
                    log_event("error", "task.cancel_failed", task=task_id, reason="ROS2_CANCEL_FAILED", detail=detail)
                    fleet_state.tasks[task_id].status = "cancelled"
                    fleet_state.add_log("warning", f"任务 {task_id} 本地已标记取消，但ROS2取消失败: {detail}")
                    return {"success": False, "message": detail}
            except HTTPException:
                # 直接透传 HTTPException 给前端
                raise
            except Exception as e:
                log_event("error", "task.cancel_failed", task=task_id, reason="ROS2_CANCEL_EXCEPTION", detail=str(e))
                fleet_state.tasks[task_id].status = "cancelled"
                fleet_state.add_log("warning", f"任务 {task_id} 本地已标记取消，但ROS2取消异常: {e}")
                return {"success": False, "message": str(e)}

    # ROS2 未连接/服务不可用：仅进行 Web 状态取消（不保证底盘停止）
    fleet_state.tasks[task_id].status = "cancelled"
    fleet_state.add_log("info", f"任务 {task_id} 已取消（ROS2未调用取消服务）")
    return {"success": True}

@app.post("/api/emergency_stop")
async def emergency_stop(robot_id: Optional[str] = None):
    """紧急停止"""
    if robot_id:
        fleet_state.add_log("warning", f"紧急停止机器人: {robot_id}")
    else:
        fleet_state.add_log("error", "紧急停止所有机器人")
    
    await fleet_state.broadcast({
        "type": "emergency_stop",
        "robot_id": robot_id
    })
    
    return {"success": True}

@app.post("/api/robots/{robot_id}/stop")
async def stop_robot(robot_id: str):
    """停止单个机器人（Web UI 兼容接口）"""
    if robot_id not in fleet_state.robots:
        raise HTTPException(status_code=404, detail="机器人不存在")
    return await emergency_stop(robot_id)

@app.get("/api/status")
async def get_status():
    """获取系统状态"""
    online_count = sum(1 for r in fleet_state.robots.values() if r.get('online'))
    
    return {
        "ros_connected": fleet_state.ros_connected,
        "total_robots": len(fleet_state.robots),
        "online_robots": online_count,
        "total_tasks": len(fleet_state.tasks),
        "waypoints_count": len(fleet_state.waypoints)
    }

@app.get("/api/logs")
async def get_logs(limit: int = 100):
    """获取日志"""
    return {"logs": fleet_state.logs[:limit]}


# ==================== WebSocket ====================

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    fleet_state.websocket_clients.append(websocket)
    fleet_state.add_log("info", "WebSocket 客户端连接")
    
    try:
        # 发送初始状态
        await websocket.send_json({
            "type": "init",
            "payload": {
                "robots": fleet_state.robots,
                "tasks": [v.dict() if hasattr(v, 'dict') else dict(v) for v in fleet_state.tasks.values()],
                "waypoints": fleet_state.waypoints,
                "map_data": fleet_state.map_data,
                "ros_connected": fleet_state.ros_connected
            }
        })
        
        while True:
            data = await websocket.receive_json()
            msg_type = data.get("type")
            payload = data.get("payload", {})
            
            if msg_type == "ping":
                await websocket.send_json({"type": "pong"})
            
            elif msg_type == "get_fleet_status":
                await websocket.send_json({
                    "type": "fleet_status",
                    "payload": fleet_state.robots
                })
            elif msg_type == "submit_task":
                req = SubmitTaskRequest(
                    waypoint_id=payload.get("waypoint_id", ""),
                    priority=payload.get("priority", 0),
                    robot_id=payload.get("robot_id"),
                    task_type=payload.get("task_type", 1),
                    site_code=payload.get("site_code", 0)
                )
                result = await create_task(req)
                await websocket.send_json({"type": "task_created", "task": result.get("task")})
            elif msg_type == "cancel_task":
                task_id = payload.get("task_id")
                if task_id:
                    await cancel_task(task_id)
            elif msg_type == "emergency_stop":
                await emergency_stop(payload.get("robot_id"))

            elif msg_type == "recall_robot":
                rid = payload.get("robot_id")
                if rid:
                    await recall_robot(rid)
                    await websocket.send_json({"type": "recall_ok", "robot_id": rid})
            elif msg_type == "remove_robot":
                rid = payload.get("robot_id")
                if rid:
                    result = await remove_robot(rid)
                    await websocket.send_json({"type": "robot_removed", "payload": {"robot_id": rid, "result": result}})
    
    except WebSocketDisconnect:
        if websocket in fleet_state.websocket_clients:
            fleet_state.websocket_clients.remove(websocket)
        fleet_state.add_log("info", "WebSocket 客户端断开")
    except Exception as e:
        log_event("error", "websocket.error", detail=str(e))
        if websocket in fleet_state.websocket_clients:
            fleet_state.websocket_clients.remove(websocket)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='FleetOS Web Backend')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='监听地址 (默认 0.0.0.0，允许同网络设备访问)')
    parser.add_argument('--port', type=int, default=8080, help='服务端口')
    parser.add_argument('--traffic-map', type=str, default=None, help='交通地图文件路径')
    args = parser.parse_args()
    
    if args.traffic_map:
        TRAFFIC_MAP_FILE = args.traffic_map
        os.environ["FLEET_TRAFFIC_MAP_FILE"] = args.traffic_map
        load_traffic_map(args.traffic_map)
    
    uvicorn.run(app, host=args.host, port=args.port)
