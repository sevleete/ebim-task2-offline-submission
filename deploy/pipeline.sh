#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
PIXI="${PIXI:-$HOME/.pixi/bin/pixi}"

cleanup() {
  trap - EXIT INT TERM   # 只收尾一次;收尾期间再按 Ctrl-C 不重入
  trap "" INT
  echo
  echo "[pipeline] 收尾:关闭左臂阻抗(臂将原地抱闸停住)…请勿再按 Ctrl-C"
  (cd "$DIR/.." && $PIXI run tmrctl --rearm left --off) || true
  echo "[pipeline] 完成。右臂仍锚定;需要全停再跑 tmrctl --rearm right --off"
}
trap cleanup EXIT INT TERM

if [ "${1:-}" = "--skip-prepare" ]; then
  echo "══════ 跳过 PREPARE(--skip-prepare)══════"
else
  echo "══════ 阶段1:PREPARE ══════"
  bash "$DIR/prepare/run_prepare.sh"
  echo; echo "══════ 等待 2s ══════"; sleep 2
fi

echo "══════ 阶段2:π0.5 推理 ══════"
echo "[pipeline] 打开左爪(上一回合可能夹着东西收的尾)"
(cd "$DIR/.." && $PIXI run tmrctl --left_gripper 1.0) || true
echo "(HOLD 建立后自动 rearm left 并自动开始执行;p 暂停,q 退出,Ctrl-C 急停)"
bash "$DIR/inference/run_client.sh" \
  --rearm-cmd "$PIXI run tmrctl --rearm left"
