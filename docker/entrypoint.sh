#!/usr/bin/env bash
set -e
cd /app
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

CFG=deploy/inference/config.yaml
if [ -n "${SERVER_HOST:-}" ] || [ -n "${SERVER_PORT:-}" ]; then
  python3 - << EOF
import os, yaml
c = yaml.safe_load(open("$CFG"))
h = os.environ.get("SERVER_HOST")
p = os.environ.get("SERVER_PORT")
if h:
    c["server"]["host"] = h
if p:
    c["server"]["port"] = int(p)
yaml.safe_dump(c, open("$CFG", "w"), allow_unicode=True, sort_keys=False)
print("[docker] server ->", c["server"]["host"], ":", c["server"]["port"])
EOF
fi

case "${1:-run}" in
  run)        exec bash deploy/pipeline_full.sh "${@:2}" ;;
  run-docked) exec bash deploy/pipeline.sh "${@:2}" ;;
  client)     exec bash deploy/inference/run_client.sh "${@:2}" ;;
  bringup)    exec bash deploy/prepare/up_and_enable_ctl.sh ;;
  viewer)     exec bash viewer-ui/run.sh --bind 0.0.0.0 --port "${VIEWER_PORT:-8090}" ;;
  tmrctl)     shift; exec python3 -m controller "$@" ;;
  health)     exec python3 docker/health.py ;;
  bash|sh)    shift; exec bash "$@" ;;
  *)          exec "$@" ;;
esac
