#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/../deploy/env_ros.sh"
export RCUTILS_LOGGING_USE_STDOUT=1
echo "[relay] DDS噪音与报错 → /tmp/relay_stderr.log"
exec python3 "$DIR/gello_relay.py" "$@" 2>>/tmp/relay_stderr.log
