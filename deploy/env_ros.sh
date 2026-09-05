#!/usr/bin/env bash
# shared ROS env setup: works on det (Humble + wired DDS config) and on the
# robot hosts (source their native tmr_env.sh). Source this, do not execute.
_ENV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$HOME/tmr_env.sh" ]; then
  source "$HOME/tmr_env.sh"
else
  if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
  elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
  fi
  if ip link show enp3s0 >/dev/null 2>&1; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="file://$_ENV_ROOT/viewer-ui/cyclonedds.xml"
  fi
fi
