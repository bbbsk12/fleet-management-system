#!/usr/bin/env python3
"""
机器人底盘检测工具 —— ROS2 节点版本。

通过订阅 ROS2 话题来发现并跟踪连接到底盘的机器人，
周期性扫描话题列表并更新各机器人的通信状态。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
import re
import json
import time
from threading import Lock
from typing import Dict, List, Optional
try:
    from .models import RobotInfo
except ImportError:
    from models import RobotInfo


class RobotDetectorNode(Node):
    """机器人检测 ROS2 节点，负责发现底盘、更新状态并发布检测结果。"""

    def __init__(self):
        super().__init__('robot_detector')

        # ---- 参数声明与读取 ----
        self.declare_parameter('zenoh_prefix', '/nav2_tb3')
        self.declare_parameter('scan_interval', 5.0)
        self.declare_parameter('namespace_pattern', r'/nav2_tb3_\d+')

        self.zenoh_prefix = self.get_parameter('zenoh_prefix').value
        self.scan_interval = self.get_parameter('scan_interval').value
        self.namespace_pattern = self.get_parameter('namespace_pattern').value

        # ---- 机器人状态存储 ----
        self.robots: Dict[str, RobotInfo] = {}
        self.robots_lock = Lock()

        # ---- 订阅者字典 ----
        self.subscriptions = {}

        # ---- QoS 配置 ----
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ---- 检测结果发布器 ----
        self.detection_pub = self.create_publisher(
            String,
            '/robot_detector/detected_robots',
            10
        )

        # ---- 定时器 ----
        self.scan_timer = self.create_timer(
            self.scan_interval,
            self.scan_robots
        )
        self.publish_timer = self.create_timer(1.0, self.publish_detection)

        self.get_logger().info(f'Robot Detector started, prefix: {self.zenoh_prefix}')
        self.get_logger().info(f'Namespace pattern: {self.namespace_pattern}')

    def scan_robots(self):
        """扫描 ROS2 话题列表，从中提取并更新已知的机器人命名空间。"""
        topic_list = self.get_topic_names_and_types()
        service_list = self.get_service_names_and_types()
        action_list = self.get_action_names_and_types()

        # ---- 从话题与服务中提取命名空间 ----
        detected_namespaces = set()

        for topic, types in topic_list:
            match = re.match(r'(/[^/]+/[^/]+)/.*', topic)
            if match:
                ns = match.group(1)
                if re.match(self.namespace_pattern, ns) or self.zenoh_prefix in ns:
                    detected_namespaces.add(ns)

        for service, types in service_list:
            match = re.match(r'(/[^/]+/[^/]+)/.*', service)
            if match:
                ns = match.group(1)
                if re.match(self.namespace_pattern, ns) or self.zenoh_prefix in ns:
                    detected_namespaces.add(ns)

        # ---- 更新机器人列表（新增 / 移除） ----
        with self.robots_lock:
            for ns in detected_namespaces:
                if ns not in self.robots:
                    self.robots[ns] = RobotInfo(
                        namespace=ns,
                        name=ns.strip('/').replace('/', '_')
                    )
                    self.get_logger().info(f'New robot detected: {ns}')
                    self._setup_subscribers(ns)

            to_remove = [ns for ns in self.robots if ns not in detected_namespaces]
            for ns in to_remove:
                self.get_logger().warn(f'Robot lost: {ns}')
                self._remove_subscribers(ns)
                del self.robots[ns]

        # ---- 更新各话题可用状态 ----
        self._update_topic_status(topic_list, service_list, action_list)

    def _setup_subscribers(self, namespace: str):
        """为指定命名空间的机器人创建所需的订阅者。"""
        if namespace in self.subscriptions:
            return

        subs = {}

        # 订阅里程计话题
        subs['odom'] = self.create_subscription(
            Odometry,
            f'{namespace}/odom',
            lambda msg, ns=namespace: self._odom_callback(ns, msg),
            self.sensor_qos
        )

        # 订阅电池状态话题
        subs['battery'] = self.create_subscription(
            BatteryState,
            f'{namespace}/battery_state',
            lambda msg, ns=namespace: self._battery_callback(ns, msg),
            self.sensor_qos
        )

        # 订阅 AMCL 定位话题
        subs['amcl'] = self.create_subscription(
            PoseWithCovarianceStamped,
            f'{namespace}/amcl_pose',
            lambda msg, ns=namespace: self._amcl_callback(ns, msg),
            self.sensor_qos
        )

        self.subscriptions[namespace] = subs

    def _remove_subscribers(self, namespace: str):
        """移除指定命名空间对应的所有订阅者。"""
        if namespace in self.subscriptions:
            for sub in self.subscriptions[namespace].values():
                self.destroy_subscription(sub)
            del self.subscriptions[namespace]

    def _update_topic_status(self, topic_list, service_list, action_list):
        """根据当前话题列表更新各机器人的功能状态标志位。"""
        with self.robots_lock:
            for ns, robot in self.robots.items():
                robot.has_odom = any(f'{ns}/odom' in t for t, _ in topic_list)
                robot.has_battery = any(f'{ns}/battery_state' in t for t, _ in topic_list)
                robot.has_tf = any(f'{ns}/tf' in t for t, _ in topic_list)
                robot.has_cmd_vel = any(f'{ns}/cmd_vel' in t for t, _ in topic_list)
                robot.has_amcl = any(f'{ns}/amcl_pose' in t for t, _ in topic_list)

                robot.has_nav2 = any(
                    f'{ns}/navigate_to_pose' in a
                    for a, _ in action_list
                )

    def _odom_callback(self, namespace: str, msg: Odometry):
        """里程计话题回调：更新机器人位置信息与更新时间戳。"""
        with self.robots_lock:
            if namespace in self.robots:
                self.robots[namespace].position = {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z,
                    'orientation': {
                        'x': msg.pose.pose.orientation.x,
                        'y': msg.pose.pose.orientation.y,
                        'z': msg.pose.pose.orientation.z,
                        'w': msg.pose.pose.orientation.w,
                    }
                }
                self.robots[namespace].last_update = time.time()

    def _battery_callback(self, namespace: str, msg: BatteryState):
        """电池状态话题回调：更新机器人电量百分比与更新时间戳。"""
        with self.robots_lock:
            if namespace in self.robots:
                self.robots[namespace].battery_percent = msg.percentage * 100
                self.robots[namespace].last_update = time.time()

    def _amcl_callback(self, namespace: str, msg: PoseWithCovarianceStamped):
        """AMCL 定位话题回调：更新机器人位置信息与更新时间戳。"""
        with self.robots_lock:
            if namespace in self.robots:
                self.robots[namespace].position = {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z,
                    'orientation': {
                        'x': msg.pose.pose.orientation.x,
                        'y': msg.pose.pose.orientation.y,
                        'z': msg.pose.pose.orientation.z,
                        'w': msg.pose.pose.orientation.w,
                    }
                }
                self.robots[namespace].last_update = time.time()

    def publish_detection(self):
        """发布当前检测结果（包含所有已发现机器人的状态快照）。"""
        with self.robots_lock:
            robots_data = [r.to_dict() for r in self.robots.values()]

        msg = String()
        msg.data = json.dumps({
            'timestamp': time.time(),
            'robot_count': len(robots_data),
            'robots': robots_data
        }, ensure_ascii=False)

        self.detection_pub.publish(msg)

    def get_robots(self) -> Dict[str, RobotInfo]:
        """获取当前所有已发现机器人的字典副本。"""
        with self.robots_lock:
            return dict(self.robots)


def main(args=None):
    """节点入口函数：初始化 ROS2、运行节点并在退出时清理资源。"""
    rclpy.init(args=args)

    node = RobotDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
