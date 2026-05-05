#!/usr/bin/env python3
"""
Zenoh 话题监听工具。

类似 ros2 topic echo 的 Zenoh 版本，支持跨 ROS_DOMAIN_ID 监听话题数据，
提供单话题监听与活跃话题列表扫描两种模式。
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
    """从 ROS2 消息的二进制头部提取时间戳（秒.纳秒）。

    参数:
        data: 消息原始字节数据。

    返回:
        格式化后的时间戳字符串，提取失败时返回 "?"。
    """
    try:
        if len(data) >= 8:
            sec = struct.unpack('<I', data[0:4])[0]
            nanosec = struct.unpack('<I', data[4:8])[0]
            return f"{sec}.{nanosec:09d}"
    except:
        pass
    return "?"


def format_size(size: int) -> str:
    """将字节大小格式化为人类可读的字符串（B、KB、MB）。

    参数:
        size: 字节数。

    返回:
        格式化后的尺寸字符串。
    """
    if size < 1024:
        return f"{size}B"
    elif size < 1024 * 1024:
        return f"{size/1024:.1f}KB"
    else:
        return f"{size/1024/1024:.1f}MB"


def listen_topic(router: str, topic: str, count: int, show_raw: bool):
    """监听指定的 Zenoh 话题并实时输出接收到的消息。

    参数:
        router: Zenoh 路由器地址。
        topic: 待监听的话题名称。
        count: 接收指定条数后自动退出（0 表示持续监听）。
        show_raw: 是否以原始十六进制格式显示数据。
    """
    # ---- 创建 Zenoh 会话 ----
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", json.dumps([router]))

    print(f"连接 Zenoh: {router}")
    session = zenoh.open(conf)
    print("✓ 已连接")

    # ---- 构建 Zenoh key 表达式 ----
    if topic.startswith('/'):
        zenoh_key = "**" + topic
    else:
        zenoh_key = "**/" + topic

    print(f"监听: {zenoh_key}")
    print("-" * 60)

    received = 0
    should_exit = False

    def on_sample(sample):
        """消息回调函数：处理接收到的 Zenoh 样本数据。"""
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
                # ---- 原始数据显示模式 ----
                print(f"  大小: {format_size(len(payload))}")
                hex_str = payload[:64].hex()
                print(f"  数据: {hex_str}{'...' if len(payload) > 64 else ''}")
            else:
                # ---- 友好数据显示模式 ----
                print(f"  大小: {format_size(len(payload))}")

                try:
                    text = payload.decode('utf-8')
                    data = json.loads(text)
                    print("  类型: JSON")
                    print(json.dumps(data, indent=4, ensure_ascii=False))
                except:
                    print(f"  类型: 二进制 (ROS2 消息)")
                    print(f"  前32字节: {payload[:32].hex()}")
                    if 'tf' in key.lower():
                        print("  (TF 数据需要 ROS2 反序列化)")

            print("-" * 60)

            if count > 0 and received >= count:
                should_exit = True

        except Exception as e:
            print(f"  错误: {e}")
            print("-" * 60)

    # ---- 订阅并等待数据 ----
    sub = session.declare_subscriber(zenoh_key, on_sample)

    print("\n等待数据... (Ctrl+C 退出)")

    try:
        while not should_exit:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n已停止")

    # ---- 清理资源 ----
    try:
        session.undeclare_subscriber(sub)
    except:
        pass
    session.close()

    print(f"共接收 {received} 条消息")


def list_active_topics(router: str, duration: int):
    """扫描指定时间段内的活跃话题并统计其频率。

    参数:
        router: Zenoh 路由器地址。
        duration: 扫描持续时间（秒）。
    """
    # ---- 创建 Zenoh 会话 ----
    conf = zenoh.Config()
    conf.insert_json5("connect/endpoints", json.dumps([router]))

    print(f"连接 Zenoh: {router}")
    session = zenoh.open(conf)
    print("✓ 已连接")

    print(f"\n扫描活跃话题 ({duration}秒)...\n")

    topics = {}

    def on_sample(sample):
        """消息回调函数：统计每个话题收到的消息数量。"""
        key = str(sample.key_expr)
        if key not in topics:
            topics[key] = 0
        topics[key] += 1

    # ---- 常用的 ROS2 话题模式列表 ----
    patterns = [
        "**/odom", "**/tf", "**/tf_static", "**/scan",
        "**/amcl_pose", "**/map", "**/battery", "**/cmd_vel",
        "**/imu", "**/joint_states", "**/plan"
    ]

    # ---- 订阅所有话题模式 ----
    subs = []
    for p in patterns:
        try:
            subs.append(session.declare_subscriber(p, on_sample))
        except:
            pass

    time.sleep(duration)

    # ---- 取消订阅 ----
    for s in subs:
        try:
            session.undeclare_subscriber(s)
        except:
            pass

    session.close()

    # ---- 按命名空间分组并输出结果 ----
    if topics:
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
    """CLI 入口函数：解析命令行参数并执行监听或扫描操作。"""
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
