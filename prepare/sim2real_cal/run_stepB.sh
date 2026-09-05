#!/usr/bin/env bash
set -eo pipefail

BASE_HOST="tmr-user@172.16.0.50"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$DIR/../../deploy/env_ros.sh"
export ROS_DOMAIN_ID=0

wait_topic() {  # $1=话题名 $2=超时秒 $3=人类描述
  echo "   等 $1 就绪（最多 $2s）..."
  for i in $(seq 1 "$2"); do
    ros2 topic list 2>/dev/null | grep -qx "$1" && { echo "   ✓ $3 就绪"; return 0; }
    sleep 1
  done
  echo "   ✗ $2s 内没看到 $1（$3 没起来）"; return 1
}

echo "[1/4] .50 起雷达服务（tmrctl，cyclone 环境；mobile_teleop 不含雷达）..."
( cd "$DIR/../.." && pixi run tmrctl --up ctl --only lidar )
wait_topic /lidar_rear/scan 30 "雷达" || {
  echo "   排查: ssh $BASE_HOST 然后看 tmr_lidar 这个 tmux 会话的日志"; exit 1; }

echo "[2/4] det 后台起 scan_merger（合并前后雷达 → /ebim/merged_cloud）..."
PYTHONPATH="$DIR:${PYTHONPATH:-}" python3 "$DIR/ebim_nav/scan_merger.py" > /tmp/scan_merger.log 2>&1 &
MERGER_PID=$!
trap 'echo; echo "收尾：停 scan_merger(pid=$MERGER_PID)"; kill "$MERGER_PID" 2>/dev/null || true' EXIT
wait_topic /ebim/merged_cloud 20 "merged_cloud" || {
  echo "   看日志: tail -50 /tmp/scan_merger.log"; exit 1; }

echo "[3/4] 三链路就绪：雷达 ✓  merged_cloud ✓  底盘 odom（假定 teleop 已在跑）"
echo "      ▶ 上车前建议先看一眼建图：另开终端 ./run.sh lmap_viz"
echo "      ▶ 21cm 精度依赖 rear_len 标定：./run.sh check_wall"

echo "[4/4] 跑 step ${*:-（完整流程，回车逐步确认）}"
echo "      安全：手柄别碰、人手放 Ctrl+C 附近、每阶段 90s 超时自动停"
cd "$DIR"
./run.sh step --auto "$@"   # 不用 exec：退出后 trap 收 scan_merger
