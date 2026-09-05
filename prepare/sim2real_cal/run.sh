#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$DIR/../../deploy/env_ros.sh"
export ROS_DOMAIN_ID=0
export PYTHONPATH="$DIR:$PYTHONPATH"

denoise() { awk '/^>>> \[rcutils/{s=1} !s{print} /^<<</{s=0}' >&2; }
py() { python3 "$@" 2> >(denoise); }

case "${1:-}" in
  scan_merger)
    py "$DIR/ebim_nav/scan_merger.py" ;;
  cloud_viz)
    shift
    py "$DIR/ebim_nav/utils/viz_merged_cloud.py" "$@" ;;
  prepare)
    py -c "import sys; sys.path.insert(0,'$DIR')
from ebim_nav.prepare_pose import run_prepare
from ebim_nav.common import load_config
sys.exit(0 if run_prepare(load_config()['approach'], dry=('--dry' in sys.argv)) else 1)" "$@" ;;
  step)
    shift
    py "$DIR/ebim_nav/step_by_step.py" "$@" ;;
  dock_ref)
    shift
    py "$DIR/ebim_nav/dock_ref.py" "$@" ;;
  dock)
    shift
    py "$DIR/ebim_nav/dock.py" "$@" ;;
  dock_viz)
    py "$DIR/ebim_nav/dock_viz.py" ;;
  pbd_record)
    py "$DIR/ebim_nav/pbd_record.py" "${2:-pbd_task}" ;;
  pbd_replay)
    shift
    py "$DIR/ebim_nav/pbd_replay.py" "$@" ;;
  pbd_viz)
    shift
    py "$DIR/ebim_nav/pbd_viz.py" "$@" ;;
  lmap_record)
    py "$DIR/ebim_nav/utils/record_frames.py" "${2:-lmap}" ;;
  lmap_viz)
    shift
    py "$DIR/ebim_nav/utils/viz_local_map.py" "$@" ;;
  check_rot)
    py "$DIR/ebim_nav/utils/check_rotation.py" ;;
  check_wall)
    py "$DIR/ebim_nav/utils/check_rear_wall.py" ;;
  *)
    echo "用法: $0 <prepare|scan_merger|cloud_viz|step|dock_ref|dock|dock_viz|pbd_record|pbd_replay|pbd_viz|lmap_record|lmap_viz|check_rot|check_wall>"; exit 1 ;;
esac
