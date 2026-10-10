#!/usr/bin/env bash
set -eo pipefail
workspace="${PERSON_FOLLOW_WS:-${HOME}/person_follow_ws}"
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"

existing_nodes="$(ros2 node list)"
while IFS= read -r node; do
  case "${node}" in
    /gazebo|/simulated_person|/follow_supervisor|/following_server|/vision_preview_fixture)
      printf 'End the existing simulation/control launch first; active node: %s\n' "${node}" >&2
      exit 1
      ;;
  esac
done <<< "${existing_nodes}"
mkdir -p "${workspace}/records"
record="${workspace}/records/vision_preview_$(date +%Y%m%d_%H%M%S).csv"
printf 'ROS_DOMAIN_ID=%s\nPreview CSV=%s\nObserve /follow_preview/cmd_vel\n' "${ROS_DOMAIN_ID}" "${record}"
exec ros2 launch person_follow_bringup vision_preview.launch.py log_path:="${record}" "$@"
