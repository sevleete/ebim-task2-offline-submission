#!/usr/bin/env bash
set -eo pipefail

BASE_HOST="tmr-user@172.16.0.50"
NAME="${1:-dock_task}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
CDX="$DIR/../../viewer-ui/cyclonedds.xml"
[ -f "$CDX" ] && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$CDX"

echo "[1/4] .50 先杀掉旧 teleop / 底盘控制进程（别的服务不动）..."
ssh "$BASE_HOST" 'bash -s' <<'EOF'
pkill -INT -f "mobile_teleop"                          || true   # 先温和 SIGINT
sleep 3
pkill -9   -f "mobile_teleop|tmrv0_2|ros2_control_node" || true   # 还没死就强杀
sleep 1
EOF

echo "[2/4] .50 后台启动 mobile_teleop（手柄 + 底盘控制栈）..."
ssh -f "$BASE_HOST" 'bash -lc "
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///home/tmr-user/cyclonedds.xml
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  nohup ros2 launch franka_bringup mobile_teleop.launch.py \
    > /tmp/mobile_teleop.log 2>&1 < /dev/null &
"'

echo "[3/4] 等 /swerve_drive_controller/cmd_vel 就绪（最多 40s）..."
for i in $(seq 1 8); do
  if timeout 20 ros2 topic list --no-daemon 2>/dev/null | grep -q "^/swerve_drive_controller/cmd_vel$"; then
    echo "      ✓ cmd_vel 已就绪"
    break
  fi
  if [ "$i" -eq 8 ]; then
    echo "      ✗ 40s 内没看到 cmd_vel。查 .50: ssh $BASE_HOST 'tail -f /tmp/mobile_teleop.log'"
    echo "        底盘若有 reflex/不动，det 上执行: (cd ../.. && pixi run tmrctl --enable base)"
    exit 1
  fi
  sleep 1
done

echo "[4/4] det 开始录制「$NAME」"
echo "      → 手柄 RB 慢档开到桌前理想位姿，到位后 Ctrl+C 保存"
echo "      → 只按 RB，别按 LB turbo；记住起点（回放要挪回这里）"
cd "$DIR"
exec ./run.sh pbd_record "$NAME"
