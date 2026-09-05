#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"

CFG="${CFG:-$DIR/config.yaml}"
HOST="$(python3 -c "import yaml;print(yaml.safe_load(open('$CFG'))['server']['host'])")"
PORT="$(python3 -c "import yaml;print(yaml.safe_load(open('$CFG'))['server']['port'])")"

# 默认直连 config 里的 server(Mode A 远程公网 IP / Mode B 本地 127.0.0.1 都适用)。
# 仅当显式设置 SERVER_SSH=user@host 时,才为服务器只放行 SSH 的场景建立本地转发,
# 此时 config 的 host 应填 127.0.0.1。不带任何内置服务器地址。
if [ -n "${SERVER_SSH:-}" ]; then
  pkill -f "L ${PORT}:localhost:${PORT}" 2>/dev/null || true
  sleep 0.5
  echo "[deploy] 经 SSH 转发 ${PORT} → $SERVER_SSH"
  ssh -f -N -o ExitOnForwardFailure=yes -o ConnectTimeout=8 \
      -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
      -L "${PORT}:localhost:${PORT}" "$SERVER_SSH"
else
  echo "[deploy] 直连推理服务器 ${HOST}:${PORT}"
fi

source "$DIR/../env_ros.sh"
cd "$DIR/../.."
echo "[deploy] DDS噪音 → /tmp/deploy_stderr.log"
exec python3 deploy/inference/robot_client.py "$@" 2>>/tmp/deploy_stderr.log
