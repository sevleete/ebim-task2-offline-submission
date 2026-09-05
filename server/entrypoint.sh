#!/usr/bin/env bash
set -e
CKPT="${CKPT:-/models/ckpt}"
PORT="${SERVER_PORT:-6060}"
[ -f "$CKPT/config.json" ] || { echo "✗ 未找到权重: $CKPT/config.json —— 用 -v <本地HF目录>:/models/ckpt:ro 挂载" >&2; exit 1; }
EXTRA=()
[ "${RTC:-on}" = "off" ] && EXTRA+=(--no-rtc)
echo "[server-image] ckpt=$CKPT port=$PORT rtc=${RTC:-on}"
exec python3 /app/infer_server.py --ckpt "$CKPT" --port "$PORT" "${EXTRA[@]}"
