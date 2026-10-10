#!/usr/bin/env bash
set -eo pipefail
workspace="${PERSON_FOLLOW_WS:-${HOME}/person_follow_ws}"
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
mkdir -p "${workspace}/records"
exec ros2 bag record -o "${workspace}/records/follow_$(date +%Y%m%d_%H%M%S)" \
  /clock /tf /tf_static /odom /cmd_vel /person/target_point /person/visible \
  /person/detected /person/depth_status /follow/person_pose /follow/target_base \
  /follow/state /follow/cmd_vel_raw /sim/target_ground_truth
