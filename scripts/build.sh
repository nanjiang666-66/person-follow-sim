#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${PERSON_FOLLOW_WS:-${HOME}/person_follow_ws}"
source /opt/ros/humble/setup.bash
if [[ -f "${workspace}/deps_install/setup.bash" ]]; then source "${workspace}/deps_install/setup.bash"; fi
if ! ros2 pkg prefix opennav_following >/dev/null 2>&1; then
  printf 'Run bash "%s/scripts/install_dependencies.sh" first.\n' "${project_dir}" >&2
  exit 1
fi
mkdir -p "${workspace}/records"
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base "${workspace}/log" build --base-paths "${project_dir}/src" \
  --build-base "${workspace}/build" --install-base "${workspace}/install" \
  --parallel-workers 1 --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source "${workspace}/install/setup.bash"
colcon --log-base "${workspace}/log" test --base-paths "${project_dir}/src" \
  --build-base "${workspace}/build" --install-base "${workspace}/install" \
  --packages-select person_follow_control person_follow_sim --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon test-result --test-result-base "${workspace}/build" --verbose
python3 -m unittest discover -s "${project_dir}/tests" -p 'test_*.py'
dpkg-query -W -f='${Package} ${Version}\n' 'ros-humble-opennav-*' 'ros-humble-nav2-*' \
  > "${workspace}/records/installed_versions.txt" 2>/dev/null || true
if [[ -f "${workspace}/records/upstream_source_commit.txt" ]]; then
  cat "${workspace}/records/upstream_source_commit.txt" >> "${workspace}/records/installed_versions.txt"
elif [[ -d "${workspace}/deps/opennav_docking/.git" ]]; then
  git -C "${workspace}/deps/opennav_docking" rev-parse HEAD >> "${workspace}/records/installed_versions.txt"
fi
printf '\nBuild ready: source %s/install/setup.bash\n' "${workspace}"
