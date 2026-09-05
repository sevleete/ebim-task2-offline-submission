#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$DIR/../deploy/env_ros.sh"
export ROS_DOMAIN_ID=0
exec python3 "$DIR/backend/server.py" "$@"
