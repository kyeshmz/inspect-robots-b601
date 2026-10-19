#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help ]]; then
  echo 'Run isolated ROS protocol tests. Source ROS and the rebotarm_msgs overlay first.'
  exit 0
fi
[[ $# == 0 ]] || { echo 'No arguments accepted except --help' >&2; exit 2; }
export ROS_DOMAIN_ID=163 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
unset ROS_DISCOVERY_SERVER ROS_STATIC_PEERS
python -c 'import rclpy, control_msgs, rebotarm_msgs' || {
  echo 'Source ROS 2 Humble and the test-only rebotarm_msgs overlay.' >&2; exit 1;
}
exec python -m pytest --ros "$ROOT/tests/ros" -v --junitxml="$ROOT/logs/ros-protocol.xml"
