#!/usr/bin/env python3
"""
Zenoh 话题监听工具
类似 ros2 topic echo，但通过 Zenoh 监听（支持跨 ROS_DOMAIN_ID）
"""

import argparse
import json
import sys
import time
import struct
from datetime import datetime

try:
    import zenoh
except ImportError:
    print("需要安装: pip install eclipse-zenoh")
    sys.exit(1)


def format_timestamp(data: bytes) -> str:
    """尝试从数据中提取时间戳"""
    try:
        # ROS2 Header: stamp (sec, nanosec) + frame_id
        if len(data) >= 8:
            sec = struct.unpack('<I', data[0:4])[0]
            nanosec = struct.unpack('<I', data[4:8])[0]
            return f"{sec}.{nanosec:09d}"
    except:
        pass
    return "?"


def format_size(size: int) -> str:
    """格式化大小"""
    if size < 1024:
        return f"{size}B"
    elif size < 1024 * 1024:
        return f"{size/1024:.1f}KB"
    else:
        return f"{size/1024/1024:.1f}MB"


def listen_topic(router: str, topic: str, count: int, show_raw: bool):
    """监听话题"""
    
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", json.dumps([router]))
    
    print(f"连接 Zenoh: {router}")
    session = zenoh.open(conf)
    print("✓ 已连接")
    
    # 构建 Zenoh key 表达式
    if topic.startswith('/'):
        zenoh_key = "**" + topic
    else:
        zenoh_key = "**/" + topic
    
    print(f"监听: {zenoh_key}")
    print("-" * 60)
    
    received = 0
    should_exit = False
    
    def on_sample(sample):
        nonlocal received, should_exit
        
        if should_exit:
            return
            
        received += 1
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        try:
            payload = sample.payload
            key = str(sample.key_expr)
            
            print(f"\n[{now}] #{received} {key}")
            
            if show_raw:
                # 显示原始数据
                print(f"  大小: {format_size(len(payload))}")
                # 显示十六进制
                hex_str = payload[:64].hex()
                print(f"  数据: {hex_str}{'...' if len(payload) > 64 else ''}")
            else:
                # 友好显示
                print(f"  大小: {format_size(len(payload))}")
                
                # 尝试解析常见格式
                try:
                    # 尝试 JSON
                    text = payload.decode('utf-8')
                    data = json.loads(text)
                    print("  类型: JSON")
                    print(json.dumps(data, indent=4, ensure_ascii=False))
                except:
                    # 二进制数据，显示摘要
                    print(f"  类型: 二进制 (ROS2 消息)")
                    print(f"  前32字节: {payload[:32].hex()}")
                    
                    # 如果是 TF，尝试解析变换数量
                    if 'tf' in key.lower():
                        # TFMessage 包含 transforms 数组
                        print("  (TF 数据需要 ROS2 反序列化)")
                        
            print("-" * 60)
            
            if count > 0 and received >= count:
                should_exit = True
                
        except Exception as e:
            print(f"  错误: {e}")
            print("-" * 60)
    
    # 订阅
    sub = session.declare_subscriber(zenoh_key, on_sample)
    
    print("\n等待数据... (Ctrl+C 退出)")
    
    try:
        while not should_exit:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n已停止")
    
    try:
        session.undeclare_subscriber(sub)
    except:
        pass
    session.close()
    
    print(f"共接收 {received} 条消息")


def list_active_topics(router: str, duration: int):
    """列出活跃的话题"""
    
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", json.dumps([router]))
    
    print(f"连接 Zenoh: {router}")
    session = zenoh.open(conf)
    print("✓ 已连接")
    
    print(f"\n扫描活跃话题 ({duration}秒)...\n")
    
    topics = {}
    
    def on_sample(sample):
        key = str(sample.key_expr)
        if key not in topics:
            topics[key] = 0
        topics[key] += 1
    
    # 订阅通配符来发现话题
    patterns = [
        "**/odom", "**/tf", "**/tf_static", "**/scan", 
        "**/amcl_pose", "**/map", "**/battery", "**/cmd_vel",
        "**/imu", "**/joint_states", "**/plan"
    ]
    
    subs = []
    for p in patterns:
        try:
            subs.append(session.declare_subscriber(p, on_sample))
        except:
            pass
    
    time.sleep(duration)
    
    for s in subs:
        try:
            session.undeclare_subscriber(s)
        except:
            pass
    
    session.close()
    
    if topics:
        # 按命名空间分组
        namespaces = {}
        for topic, count in topics.items():
            parts = topic.split('/')
            if len(parts) >= 1:
                ns = parts[0] if parts[0] else (parts[1] if len(parts) > 1 else 'unknown')
                if ns not in namespaces:
                    namespaces[ns] = []
                namespaces[ns].append((topic, count))
        
        print("活跃话题:")
        print("=" * 60)
        
        for ns in sorted(namespaces.keys()):
            print(f"\n📁 {ns}")
            for topic, count in sorted(namespaces[ns]):
                freq = count / duration
                print(f"   {topic} ({freq:.1f} Hz, {count} 条)")
        
        print()
        print(f"共 {len(topics)} 个话题")
    else:
        print("未发现活跃话题")


def main():
    parser = argparse.ArgumentParser(
        description='Zenoh 话题监听工具 (类似 ros2 topic echo)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 监听 hefbot1 的 tf
  zenoh_echo -t hefbot1/tf
  
  # 监听所有 tf
  zenoh_echo -t tf
  
  # 接收 10 条消息后退出
  zenoh_echo -t odom -n 10
  
  # 列出活跃话题
  zenoh_echo -l
  
  # 显示原始数据
  zenoh_echo -t scan --raw
        """
    )
    
    parser.add_argument('--router', '-r', default='tcp/127.0.0.1:7447',
                        help='Zenoh 路由器地址')
    parser.add_argument('--topic', '-t',
                        help='要监听的话题')
    parser.add_argument('--count', '-n', type=int, default=0,
                        help='接收 N 条后退出')
    parser.add_argument('--raw', action='store_true',
                        help='显示原始数据')
    parser.add_argument('--list', '-l', action='store_true',
                        help='列出活跃话题')
    parser.add_argument('--duration', '-d', type=int, default=5,
                        help='扫描时长(秒)，配合 -l 使用')
    
    args = parser.parse_args()
    
    if args.list:
        list_active_topics(args.router, args.duration)
    elif args.topic:
        listen_topic(args.router, args.topic, args.count, args.raw)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
