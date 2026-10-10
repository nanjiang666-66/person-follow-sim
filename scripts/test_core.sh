#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "${project_dir}/.cache"
"${CXX:-g++}" -std=c++17 -Wall -Wextra -Wpedantic \
  -I"${project_dir}/src/person_follow_control/include" \
  "${project_dir}/src/person_follow_control/test/test_control_math.cpp" \
  -o "${project_dir}/.cache/test_control_math"
"${project_dir}/.cache/test_control_math"
"${CXX:-g++}" -std=c++17 -Wall -Wextra -Wpedantic \
  -I"${project_dir}/src/person_follow_sim/include" \
  "${project_dir}/src/person_follow_sim/test/test_scenarios.cpp" \
  -o "${project_dir}/.cache/test_scenarios"
"${project_dir}/.cache/test_scenarios"
python3 -m unittest discover -s "${project_dir}/tests" -p 'test_*.py'
for script in "${project_dir}"/scripts/*.sh; do bash -n "${script}"; done
