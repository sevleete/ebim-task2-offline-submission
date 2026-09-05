#!/usr/bin/env bash
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/../.." && pwd)"
PIXI="${PIXI:-$HOME/.pixi/bin/pixi}"
TMRCTL="$PIXI run tmrctl"
RETRY="${RETRY:-3}"
cd "$ROOT"

try() {  # try <描述> <提示> <命令...>
  local desc="$1" hint="$2"; shift 2
  local n=1
  while true; do
    echo; echo "━━━ $desc $([ $n -gt 1 ] && echo "(第 $n 次)")━━━"
    "$@" && return 0
    n=$((n+1))
    if [ "$n" -gt "$RETRY" ]; then
      echo "✗✗ [$desc] 重试 $RETRY 次仍失败,中止。"
      [ -n "$hint" ] && echo "   提示:$hint"
      exit 1
    fi
    echo "   ↻ 5s 后重试…"; sleep 5
  done
}

H_ARM="查 Desk 16.11/16.12 是否 解锁关节+Activate FCI,臂急停是否旋开"
H_BOX="查 16.10 Dashboard 的 Drive/Spine 是否红灯,车体急停是否旋开"

try "up base"       "$H_BOX" $TMRCTL --up ctl --only base
try "up zed"        "ZED USB 掉线的话去 .50 重插" $TMRCTL --up ctl --only zed
try "up spine"      "$H_BOX" $TMRCTL --up ctl --only spine
try "up grippers"   ""       $TMRCTL --up ctl --only grippers
try "up arms"       "$H_ARM" $TMRCTL --up ctl --only arms
try "up wrist_cams" ""       $TMRCTL --up ctl --only wrist_cams

try "enable left_arm"  "$H_ARM" $TMRCTL --enable left_arm
try "enable right_arm" "$H_ARM" $TMRCTL --enable right_arm
try "enable spine"     "$H_BOX" $TMRCTL --enable spine
try "enable grippers"  ""       $TMRCTL --enable grippers
try "enable base"      "$H_BOX" $TMRCTL --enable base

echo; echo "━━━ 相机自查 ━━━"
set +u   # ROS setup.bash 内部引用未定义变量,与 set -u 冲突
source "$DIR/../env_ros.sh"
for t in /head_camera/zed/rgb/color/rect/image/compressed \
         /wrist_camera_left/color/image_raw/compressed \
         /wrist_camera_right/color/image_raw/compressed; do
  python3 "$DIR/wait_topic.py" "$t" image 60 5 2>/dev/null || { echo "  ✗ $t 断流"; exit 1; }
done

echo; echo "✓✓ UP+ENABLE 全部就绪(base/zed/spine/grippers/arms/wrist_cams;lidar 由车段 run_stepB 自起)"
echo "   下一步: bash deploy/pipeline_full.sh(整机:车趋近→prepare→推理) 或 bash deploy/pipeline.sh(车已到位)"
