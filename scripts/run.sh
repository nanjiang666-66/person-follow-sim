#!/usr/bin/env bash
set -eo pipefail
workspace="${PERSON_FOLLOW_WS:-${HOME}/person_follow_ws}"
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
record_scenario="${1:-stationary}"
scenario="${record_scenario}"
# Use a separate CSV name for the intentional supervisor-exit test.
if [[ "${record_scenario}" == normal_exit ]]; then scenario=away; fi
if [[ $# -gt 0 ]]; then shift; fi
mkdir -p "${workspace}/records"
record="${workspace}/records/${record_scenario}_$(date +%Y%m%d_%H%M%S).csv"
printf 'ROS_DOMAIN_ID=%s\nCSV=%s\n' "${ROS_DOMAIN_ID}" "${record}"
exec ros2 launch person_follow_bringup simulation.launch.py scenario:="${scenario}" \
  log_path:="${record}" "$@"
