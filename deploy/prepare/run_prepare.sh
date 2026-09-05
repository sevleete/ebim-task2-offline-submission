#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/../.." && pwd)"
PIXI="${PIXI:-$HOME/.pixi/bin/pixi}"
TMRCTL="$PIXI run tmrctl"

LEFT_PREPARE="0.4122 -0.6906 -0.2393 -2.5893 0.0927 2.1637 1.3100"
RIGHT_PREPARE="-0.8325 -0.7093 0.2961 -2.5254 0.3770 2.0277 -1.8413"
LEFT_GELLO_POSE="-1.4784 -0.6345 1.6843 -2.6402 1.2870 1.5071 -1.5399"
SPINE_UP=0.6
SPINE_WORK=0.4

source "$DIR/../env_ros.sh"
cd "$ROOT"

step() { echo; echo "━━━ $* ━━━"; }

step "① 升降柱 → ${SPINE_UP}m"
$TMRCTL --spine $SPINE_UP

step "② 双臂阻抗关 + PTP 到 prepare"
$TMRCTL --rearm left --off
$TMRCTL --rearm right --off
$TMRCTL --left_arm_joint  $LEFT_PREPARE
$TMRCTL --right_arm_joint $RIGHT_PREPARE

step "③ 右臂锁死(relay 快照 → rearm → 锚定后停 relay)"
$TMRCTL --rearm right --off
pkill -f "gello_rela[y].py" 2>/dev/null || true
sleep 1
launch_relay() {
  setsid bash -c "cd '$ROOT/teleop' && exec bash run_relay.sh" \
    >/tmp/relay_stdout.log 2>&1 </dev/null &
  disown 2>/dev/null || true
}
ok=""
for attempt in 1 2 3; do
  launch_relay
  sleep 4
  if ! pgrep -f "gello_rela[y].py" >/dev/null; then
    echo "   relay 进程死了(偶发rclpy崩溃),第 $attempt 次重拉…"
    continue
  fi
  if python3 "$DIR/wait_topic.py" /right/gello/joint_states joint 40 3 2>/dev/null; then
    ok=1; break
  fi
  echo "   relay 在跑但 40s 没等到数据,重拉(第 $attempt 次)…"
  pkill -f "gello_rela[y].py" 2>/dev/null || true
  sleep 1
done
[ -n "$ok" ] || { echo "✗ relay 三次都没起来,查 /tmp/relay_stdout.log"; exit 1; }
$TMRCTL --rearm right
sleep 2
pkill -f "gello_rela[y].py" 2>/dev/null || true
echo "   右臂已锚定,relay 已功成身退"

step "④ 左臂 PTP → 左 GELLO 起始位形(常量)"
$TMRCTL --left_arm_joint $LEFT_GELLO_POSE

step "⑤ 升降柱 → ${SPINE_WORK}m"
$TMRCTL --spine $SPINE_WORK

echo; echo "✓ PREPARE 完成:右臂已锁死,左臂在 gello 起始位形,升降柱 ${SPINE_WORK}m"
