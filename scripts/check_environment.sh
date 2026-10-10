#!/usr/bin/env bash
set -eo pipefail
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'Missing /opt/ros/humble/setup.bash\n' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
workspace="${PERSON_FOLLOW_WS:-${HOME}/person_follow_ws}"
printf '\n--- Loaded ROS workspaces ---\n'
printf '/opt/ros/humble\n'
for overlay in deps_install install; do
  if [[ -f "${workspace}/${overlay}/setup.bash" ]]; then
    source "${workspace}/${overlay}/setup.bash"
    printf '%s\n' "${workspace}/${overlay}"
  fi
done
printf '\n--- Operating system ---\n'
cat /etc/os-release
printf '\n--- ROS and Gazebo ---\n'
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-unknown}"
gazebo --version || true
printf '\n--- Tools ---\n'
for executable in colcon cmake g++ xacro rosdep git; do
  command -v "${executable}" || true
done
printf '\n--- ROS package availability ---\n'
for package in gazebo_ros gazebo_plugins xacro robot_state_publisher rviz2 nav2_lifecycle_manager opennav_following opennav_following_msgs; do
  if prefix="$(ros2 pkg prefix "${package}" 2>/dev/null)"; then
    printf '%s: %s\n' "${package}" "${prefix}"
  else
    printf '%s: not found in loaded ROS workspaces\n' "${package}"
  fi
done
printf '\n--- Following package candidates ---\n'
apt-cache policy ros-humble-opennav-following ros-humble-opennav-following-msgs ros-humble-opennav-docking ros-humble-nav2-graceful-controller
printf '\n--- Shared project ---\n'
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
printf '%s\n' "${project_dir}"
printf '\n--- Existing graph (if running) ---\n'
timeout 8 ros2 node list || true
