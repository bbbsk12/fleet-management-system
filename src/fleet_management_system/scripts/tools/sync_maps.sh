#!/bin/bash
# ============================================================
# 地图同步脚本 - 将地图同步到所有底盘
# ============================================================
# 使用方法:
#   ./scripts/tools/sync_maps.sh [map_dir] [robot_ips...]
#
# 示例:
#   ./scripts/tools/sync_maps.sh
#   ./scripts/tools/sync_maps.sh src/fleet_management_system/maps/map0
#   ./scripts/tools/sync_maps.sh src/fleet_management_system/maps/map0 192.168.1.101 192.168.1.102
# ============================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 默认配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
MAP_DIR="${1:-$PROJECT_DIR/maps/map0}"
ROBOT_USER="${ROBOT_USER:-robot}"
DEFAULT_ROBOTS=("192.168.1.101" "192.168.1.102")

# 检查地图目录
if [ ! -d "$MAP_DIR" ]; then
    echo -e "${RED}错误: 地图目录不存在: $MAP_DIR${NC}"
    exit 1
fi

# 检查地图文件
YAML_FILE=$(find "$MAP_DIR" -name "*.yaml" -type f | head -1)
if [ -z "$YAML_FILE" ]; then
    echo -e "${RED}错误: 未找到地图YAML文件${NC}"
    exit 1
fi

echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}地图同步工具${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "地图目录: $MAP_DIR"
echo "地图文件: $YAML_FILE"
echo ""

# 获取机器人IP列表
if [ $# -gt 1 ]; then
    shift
    ROBOTS=("$@")
else
    ROBOTS=("${DEFAULT_ROBOTS[@]}")
fi

echo "目标底盘:"
for robot in "${ROBOTS[@]}"; do
    echo "  - $robot"
done
echo ""

# 确认同步
read -p "确认同步? [y/N] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "已取消"
    exit 0
fi

# 同步到每个底盘
for robot in "${ROBOTS[@]}"; do
    echo ""
    echo -e "${YELLOW}同步到 $robot...${NC}"
    
    # 检查连接
    if ! ping -c 1 -W 2 "$robot" &> /dev/null; then
        echo -e "${RED}  ✗ 无法连接到 $robot${NC}"
        continue
    fi
    
    # 创建目录
    ssh "$ROBOT_USER@$robot" "mkdir -p ~/maps/current" 2>/dev/null || {
        echo -e "${RED}  ✗ SSH连接失败 (尝试使用当前用户)${NC}"
        ssh "$robot" "mkdir -p ~/maps/current" 2>/dev/null || {
            echo -e "${RED}  ✗ 无法创建目录${NC}"
            continue
        }
        ROBOT_USER="$USER"
    }
    
    # 同步文件
    if rsync -avz --delete "$MAP_DIR/" "$ROBOT_USER@$robot:~/maps/current/" 2>/dev/null; then
        echo -e "${GREEN}  ✓ 同步成功${NC}"
        
        # 验证
        REMOTE_YAML=$(ssh "$ROBOT_USER@$robot" "ls ~/maps/current/*.yaml 2>/dev/null | head -1")
        if [ -n "$REMOTE_YAML" ]; then
            echo "  地图文件: $REMOTE_YAML"
        fi
    else
        echo -e "${RED}  ✗ 同步失败${NC}"
    fi
done

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}同步完成!${NC}"
echo -e "${GREEN}============================================${NC}"
