#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/.." && pwd)"
PIXI="${PIXI:-$HOME/.pixi/bin/pixi}"

SKIP_APPROACH=""
DEBUG_ONLY=""            # --dry/--only 是调试,车段跑完就停,不进推理
STEP_ARGS=()
for a in "$@"; do
  case "$a" in
    --skip-approach) SKIP_APPROACH=1 ;;
    --dry|--only)    DEBUG_ONLY=1; STEP_ARGS+=("$a") ;;
    *) STEP_ARGS+=("$a") ;;
  esac
done

if [ -z "$SKIP_APPROACH" ]; then
  echo "══════ 阶段A:车辆趋近(spine↑+双臂prepare → 转90° → 倒到墙25cm → 右移 → dock)══════"
  echo "[full] 底盘使能三部曲(幂等,清掉可能残留的 reflex)…"
  (cd "$ROOT" && $PIXI run tmrctl --enable base) \
    || { echo "✗ 底盘使能失败——base 服务起了吗? pixi run tmrctl --up ctl --only base"; exit 1; }
  bash "$ROOT/prepare/sim2real_cal/run_stepB.sh" "${STEP_ARGS[@]}"
  if [ -n "$DEBUG_ONLY" ]; then
    echo; echo "══════ 调试模式(--dry/--only):车段结束,不进部署段 ══════"; exit 0
  fi
  echo; echo "✓ 车已到位(桌前标称作业位姿)。等待 2s 进入部署段…"; sleep 2
else
  echo "══════ 跳过阶段A(--skip-approach,车已到位)══════"
fi

echo "══════ 阶段B:deploy(车到位后 prepare 五步 + π0.5 推理)══════"
exec bash "$DIR/pipeline.sh"
