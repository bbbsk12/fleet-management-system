#!/usr/bin/env python3

"""
FleetOS 总启动文件
启动核心节点和Web上位机
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('fleet_management_system')
    
    # 获取工作空间根目录（更可靠）
    # get_package_prefix -> <ws>/install/fleet_management_system
    # workspace_root -> <ws>
    pkg_prefix = get_package_prefix('fleet_management_system')
    workspace_root = os.path.dirname(os.path.dirname(pkg_prefix))
    
    # 参数定义
    traffic_map_file = LaunchConfiguration('traffic_map_file')
    use_web = LaunchConfiguration('use_web')
    web_port = LaunchConfiguration('web_port')
    zenoh_bridge_config = LaunchConfiguration('zenoh_bridge_config')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    zenoh_router = LaunchConfiguration('zenoh_router')

    scheduler_verbose_log = LaunchConfiguration('scheduler_verbose_log')
    persist_log_enabled = LaunchConfiguration('persist_log_enabled')
    persist_log_dir = LaunchConfiguration('persist_log_dir')
    persist_log_verbose_info = LaunchConfiguration('persist_log_verbose_info')

    # 底盘任务执行超时参数
    chassis_handshake_timeout_sec = LaunchConfiguration('chassis_handshake_timeout_sec')
    chassis_exec_timeout_sec = LaunchConfiguration('chassis_exec_timeout_sec')
    chassis_max_retries = LaunchConfiguration('chassis_max_retries')

    waypoint_acceptance_radius = LaunchConfiguration('waypoint_acceptance_radius')
    traffic_segment_lateral_max = LaunchConfiguration('traffic_segment_lateral_max')
    
    # 默认地图路径（安装后从 share 目录读取）
    default_map = os.path.join(pkg_share, 'maps', 'map0', 'rmf_map0.yaml')
    default_zenoh_bridge_config = os.path.join(
        pkg_share, 'config', 'zenoh', 'host-bridge.json5')

    installed_web_backend_path = os.path.join(
        pkg_share, 'web_ui', 'backend', 'server_ros2.py')
    source_web_backend_path = os.path.join(
        workspace_root, 'apps', 'fleet_web_ui', 'backend', 'server_ros2.py')
    web_backend_path = (
        installed_web_backend_path
        if os.path.exists(installed_web_backend_path)
        else source_web_backend_path
    )

    # 运行时数据目录：持久化日志和 Web UI 设置文件
    # 注意：pkg_share 通常位于 install/，可能是只读；运行时可写数据应放到 workspace_root 下。
    default_persist_dir = os.path.join(workspace_root, 'test_logs')
    runtime_dir = os.path.join(workspace_root, 'runtime')
    default_settings_file = os.path.join(runtime_dir, 'webui_settings.json')
    
    return LaunchDescription([
        # ============ 参数声明 ============
        DeclareLaunchArgument(
            'traffic_map_file',
            default_value=default_map,
            description='交通图文件路径'
        ),
        
        DeclareLaunchArgument(
            'use_web',
            default_value='true',
            description='是否启动Web上位机'
        ),
        
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='Web后端端口'
        ),
        
        DeclareLaunchArgument(
            'zenoh_bridge_config',
            default_value=default_zenoh_bridge_config,
            description='zenoh-bridge-ros2dds 配置文件路径（json5）'
        ),

        DeclareLaunchArgument(
            'ros_domain_id',
            default_value='0',
            description='ROS_DOMAIN_ID（默认 0；避免传空导致 bridge 启动失败）'
        ),

        DeclareLaunchArgument(
            'zenoh_router',
            default_value='',
            description='ZENOH_ROUTER（留空则不设置；需配合 bridge config 使用）'
        ),

        DeclareLaunchArgument(
            'scheduler_verbose_log',
            default_value='false',
            description='是否启用调度详细日志（fleet_manager）'
        ),

        DeclareLaunchArgument(
            'persist_log_enabled',
            default_value='true',
            description='是否启用结构化落盘日志'
        ),

        DeclareLaunchArgument(
            'persist_log_dir',
            default_value=default_persist_dir,
            description='结构化落盘日志目录（建议绝对路径）'
        ),

        DeclareLaunchArgument(
            'persist_log_verbose_info',
            default_value='false',
            description='是否在持久化日志里包含 INFO（会更大）'
        ),

        DeclareLaunchArgument(
            'chassis_handshake_timeout_sec',
            default_value='5.0',
            description='底盘握手超时（秒）'
        ),

        DeclareLaunchArgument(
            'chassis_exec_timeout_sec',
            default_value='30.0',
            description='底盘执行超时（秒，握手成功后计时）'
        ),

        DeclareLaunchArgument(
            'chassis_max_retries',
            default_value='3',
            description='底盘握手超时重试次数'
        ),

        DeclareLaunchArgument(
            'waypoint_acceptance_radius',
            default_value='0.5',
            description='机器人到达航点的接受半径（米）'
        ),

        DeclareLaunchArgument(
            'traffic_segment_lateral_max',
            default_value='1.25',
            description='航段横向偏差最大阈值（米）'
        ),

        # Ensure runtime dir exists for web settings
        ExecuteProcess(
            cmd=['bash', '-lc', f'mkdir -p "{runtime_dir}"'],
            output='screen'
        ),
        
        # ============ Zenoh Bridge ============
        LogInfo(msg='>>> 启动 Zenoh Bridge...'),

        # Wait for local zenohd endpoint to avoid startup race:
        # bridge starts too early -> routes may flap and remote entities retire.
        ExecuteProcess(
            cmd=['bash', '-lc',
                 'for i in {1..60}; do '
                 '(echo > /dev/tcp/127.0.0.1/7447) >/dev/null 2>&1 && exit 0; '
                 'sleep 1; '
                 'done; '
                 'echo "[launch.user] zenohd tcp/127.0.0.1:7447 not ready after 60s" >&2; '
                 'exit 1'],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['zenoh-bridge-ros2dds', '-c', zenoh_bridge_config],
            name='zenoh_bridge',
            output='screen',
            additional_env={
                'ROS_DOMAIN_ID': ros_domain_id,
                'ZENOH_ROUTER': zenoh_router,
            }
        ),
        
        # ============ 核心节点 ============
        LogInfo(msg='>>> 启动 Fleet Monitor...'),
        
        Node(
            package='fleet_monitor',
            executable='fleet_monitor_node',
            name='fleet_monitor',
            output='screen',
            parameters=[{
                'traffic_map_file': traffic_map_file
            }],
            additional_env={
                'ROS_DOMAIN_ID': ros_domain_id,
                'ZENOH_ROUTER': zenoh_router,
            }
        ),
        
        LogInfo(msg='>>> 启动 Fleet Manager...'),
        
        Node(
            package='fleet_manager',
            executable='fleet_manager_node',
            name='fleet_manager',
            output='screen',
            parameters=[{
                'traffic_map_file': traffic_map_file,
                'waypoint_acceptance_radius': waypoint_acceptance_radius,
                'traffic_segment_lateral_max': traffic_segment_lateral_max,
                'scheduler_verbose_log': scheduler_verbose_log,
                'persist_log_enabled': persist_log_enabled,
                'persist_log_dir': persist_log_dir,
                'persist_log_verbose_info': persist_log_verbose_info,
                'chassis_handshake_timeout_sec': chassis_handshake_timeout_sec,
                'chassis_exec_timeout_sec': chassis_exec_timeout_sec,
                'chassis_max_retries': chassis_max_retries,
            }]
            ,
            additional_env={
                'ROS_DOMAIN_ID': ros_domain_id,
                'ZENOH_ROUTER': zenoh_router,
            }
        ),
        
        # ============ Web上位机（可选）============
        LogInfo(msg='>>> 启动 Web Backend...', 
                condition=IfCondition(use_web)),
        
        ExecuteProcess(
            cmd=[
                'python3',
                web_backend_path,
                '--port', web_port,
                '--traffic-map', traffic_map_file
            ],
            name='web_backend',
            output='screen',
            condition=IfCondition(use_web),
            additional_env={
                'PYTHONUNBUFFERED': '1',
                'ROS_DOMAIN_ID': ros_domain_id,
                'ZENOH_ROUTER': zenoh_router,
                'FLEET_WEB_SETTINGS_FILE': default_settings_file,
            }
        ),
    ])
