#!/usr/bin/env bash
if [ "$1" = "run" ] && [ "$2" = "tmrctl" ]; then
  shift 2
  cd /app && exec python3 -m controller "$@"
fi
echo "pixi shim: only 'pixi run tmrctl ...' is supported" >&2
exit 1
