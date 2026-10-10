#!/usr/bin/env bash
# Copy existing guest records to the shared project for offline review.
set -eo pipefail
export LC_ALL=C
shopt -s nullglob

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${PERSON_FOLLOW_WS:-${HOME}/person_follow_ws}"
record_dir="${workspace}/records"
output_dir="${project_dir}/records/incoming"

if [[ ! -d "${record_dir}" ]]; then
  printf 'Record directory not found: %s\n' "${record_dir}" >&2
  exit 1
fi
mkdir -p "${output_dir}"
count=0
for scenario in stationary away approach lateral turn occlusion invalid_depth normal_exit vision_preview; do
  # run.sh uses fixed-width YYYYMMDD_HHMMSS names; glob order gives the latest.
  candidates=("${record_dir}/${scenario}_"*.csv)
  if (( ${#candidates[@]} == 0 )); then
    # Additional stage-specific recordings are optional until performed.
    if [[ "${scenario}" == normal_exit || "${scenario}" == vision_preview ]]; then continue; fi
    printf 'Not found: %s CSV\n' "${scenario}"
    continue
  fi
  latest="${candidates[${#candidates[@]}-1]}"
  cp -- "${latest}" "${output_dir}/"
  count=$((count+1))
  printf 'Copied: %s\n' "${latest##*/}"
done
for metadata in installed_versions.txt upstream_source_commit.txt; do
  if [[ -f "${record_dir}/${metadata}" ]]; then
    cp -- "${record_dir}/${metadata}" "${output_dir}/"
  fi
done
printf 'Collected CSV files: %s\nShared directory: %s\n' "${count}" "${output_dir}"
printf 'Collection only; CSV copying does not confirm acceptance.\n'
if (( count == 0 )); then exit 1; fi
