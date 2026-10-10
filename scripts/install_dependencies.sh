#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
workspace="${PERSON_FOLLOW_WS:-${HOME}/person_follow_ws}"
upstream_commit="9d7819a214495f395f5a06244129728cbaed2bc8"
sudo apt-get update
sudo apt-get install -y build-essential cmake git python3-colcon-common-extensions python3-rosdep \
  ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-nav2-bringup ros-humble-nav2-graceful-controller \
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-msgs ros-humble-tf2-py ros-humble-tf2-ros-py \
  ros-humble-visualization-msgs ros-humble-rosbag2
printf '\n--- Installed TF packages ---\n'
dpkg-query -W -f='${Package} ${Version}\n' \
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
tf2_ros_prefix="$(ros2 pkg prefix tf2_ros)"
if [[ ! -f "${tf2_ros_prefix}/include/tf2_ros/tf2_ros/buffer_interface.hpp" && \
      ! -f "${tf2_ros_prefix}/include/tf2_ros/buffer_interface.hpp" ]]; then
  printf 'tf2_ros buffer_interface.hpp is still unavailable after apt install.\n' >&2
  printf 'Send the TF package versions and apt output before rebuilding.\n' >&2
  exit 1
fi
candidate="$(LC_ALL=C apt-cache policy ros-humble-opennav-following | awk '/Candidate:/ {print $2}')"
if [[ -n "${candidate}" && "${candidate}" != '(none)' ]]; then
  sudo apt-get install -y ros-humble-opennav-following ros-humble-opennav-following-msgs
else
  printf '\nFollowing binary unavailable; building official Humble source %s\n' "${upstream_commit}"
  source "${project_dir}/scripts/prepare_upstream.sh"
  prepare_upstream "${project_dir}" "${workspace}" "${upstream_commit}"
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
  rosdep update --rosdistro humble
  rosdep install --from-paths "${upstream}" --ignore-src --rosdistro humble \
    --dependency-types build --dependency-types buildtool --dependency-types exec -y
  export CMAKE_BUILD_PARALLEL_LEVEL=2
  colcon --log-base "${workspace}/deps_log" build --base-paths "${upstream}" \
    --build-base "${workspace}/deps_build" --install-base "${workspace}/deps_install" \
    --parallel-workers 1 --packages-up-to opennav_following --cmake-clean-cache \
    --cmake-args -DBUILD_TESTING=OFF
  source "${workspace}/deps_install/setup.bash"
fi
ros2 pkg prefix opennav_following
ros2 pkg prefix opennav_following_msgs
printf '\nDependencies ready. Next run scripts/build.sh.\n'
