#!/usr/bin/env bash
# Sourced by install_dependencies.sh. Sets upstream to the verified source directory.
prepare_upstream() {
  local project_dir="$1"
  local workspace="$2"
  local upstream_commit="$3"
  local upstream_url='https://github.com/open-navigation/opennav_docking.git'
  local cached_source="${project_dir}/third_party/opennav_docking"
  local trusted_upload_pack
  upstream="${workspace}/deps/opennav_docking"
  # A failed clone may leave an incomplete directory. Preserve it and use a new path.
  if [[ ! -d "${upstream}/.git" ]]; then
    upstream="${workspace}/deps/opennav_docking-${upstream_commit}"
  fi
  mkdir -p "${workspace}/deps"
  if [[ ! -e "${upstream}" ]]; then
    if [[ -d "${cached_source}/.git" ]]; then
      [[ "$(git -c safe.directory="${cached_source}" -C "${cached_source}" rev-parse HEAD)" == "${upstream_commit}" ]] || {
        printf 'Shared source is not the locked commit: %s\n' "${cached_source}" >&2; return 1;
      }
      [[ "$(git -c safe.directory="${cached_source}" -C "${cached_source}" remote get-url origin)" == "${upstream_url}" ]] || {
        printf 'Unexpected shared source remote: %s\n' "${cached_source}" >&2; return 1;
      }
      printf '\nUsing verified shared source: %s\n' "${cached_source}"
      # Regular local Git transport avoids hard links and ownership copying across HGFS.
      # Local transport starts a separate upload-pack process. Pass the two exact
      # trusted paths to that process explicitly; clone's -c settings may be cleared.
      printf -v trusted_upload_pack 'git -c safe.directory=%q -c safe.directory=%q upload-pack' \
        "${cached_source}" "${cached_source}/.git"
      git -c safe.directory="${cached_source}" -c safe.directory="${cached_source}/.git" \
        clone --no-local --upload-pack="${trusted_upload_pack}" -- "${cached_source}" "${upstream}" || return 1
      git -C "${upstream}" remote set-url origin "${upstream_url}" || return 1
    else
      git -c http.version=HTTP/1.1 clone --branch humble --depth 1 \
        "${upstream_url}" "${upstream}" || return 1
    fi
  fi
  [[ -d "${upstream}/.git" ]] || {
    printf 'Incomplete source directory; preserved without overwriting: %s\n' "${upstream}" >&2; return 1;
  }
  [[ "$(git -C "${upstream}" remote get-url origin)" == "${upstream_url}" ]] || {
    printf 'Unexpected upstream remote at %s\n' "${upstream}" >&2; return 1;
  }
  [[ -z "$(git -C "${upstream}" status --porcelain)" ]] || {
    printf 'Upstream checkout has local changes. Preserve them before retrying.\n' >&2; return 1;
  }
  # A complete local copy of the pinned commit needs no GitHub fetch on retries.
  if ! git -C "${upstream}" cat-file -e "${upstream_commit}^{commit}" 2>/dev/null; then
    git -C "${upstream}" -c http.version=HTTP/1.1 fetch --depth 1 origin "${upstream_commit}" || return 1
  fi
  git -C "${upstream}" checkout --detach "${upstream_commit}" || return 1
  git -C "${upstream}" fsck --connectivity-only || return 1
  mkdir -p "${workspace}/records"
  printf 'opennav_docking source commit: %s\n' "${upstream_commit}" > "${workspace}/records/upstream_source_commit.txt"
}
