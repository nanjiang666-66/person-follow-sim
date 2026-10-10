#!/usr/bin/env bash
# Request normal shutdown of the sole supervisor executable in this workspace.
# Run during the normal_exit Gazebo test; keep the launch and monitor running.
set -eo pipefail

if [[ "$(uname -s)" != Linux ]]; then
  printf 'Run this script in the Ubuntu guest.\n' >&2
  exit 1
fi
workspace="${PERSON_FOLLOW_WS:-${HOME}/person_follow_ws}"
executable="$(readlink -f -- "${workspace}/install/person_follow_control/lib/person_follow_control/follow_supervisor")"
if [[ ! -x "${executable}" ]]; then
  printf 'Supervisor executable not found in %s.\n' "${workspace}" >&2
  exit 1
fi

pids=()
for process_entry in /proc/[0-9]*/exe; do
  resolved="$(readlink -f -- "${process_entry}" 2>/dev/null)" || continue
  if [[ "${resolved}" == "${executable}" ]]; then
    process_id="${process_entry#/proc/}"
    pids+=("${process_id%/exe}")
  fi
done
if (( ${#pids[@]} != 1 )); then
  printf 'Expected exactly one supervisor in this workspace; found %s.\n' "${#pids[@]}" >&2
  exit 1
fi
supervisor_pid="${pids[0]}"
# Check the executable again before signalling the selected process.
if [[ "$(readlink -f -- "/proc/${supervisor_pid}/exe" 2>/dev/null)" != "${executable}" ]]; then
  printf 'Selected process changed; no signal sent.\n' >&2
  exit 1
fi
printf 'Sending SIGINT to supervisor PID %s: %s\n' "${supervisor_pid}" "${executable}"
kill -INT -- "${supervisor_pid}"
for (( attempt=0; attempt<50; attempt++ )); do
  if ! kill -0 -- "${supervisor_pid}" 2>/dev/null; then
    printf 'Supervisor exited. Continue observing Gazebo and recording for 5 seconds.\n'
    printf 'Process exit alone does not confirm zero commands or physical stopping.\n'
    exit 0
  fi
  sleep 0.1
done
printf 'Supervisor still exists after 5 seconds; inspect the launch log. No further signal sent.\n' >&2
exit 1
