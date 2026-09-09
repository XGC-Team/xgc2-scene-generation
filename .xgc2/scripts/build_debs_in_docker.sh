#!/usr/bin/env bash
# shellcheck disable=SC2016,SC1004
set -euo pipefail
# Literal scripts below are expanded by Bash inside the build container.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
export PACKAGE_VERSION="${PACKAGE_VERSION:-$(awk '/^version:/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml")}"
[[ -n "${PACKAGE_VERSION}" ]] || { echo "package version is missing" >&2; exit 1; }
export INSTALL_CHECK="${INSTALL_CHECK:-true}"
container_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-install-check) export INSTALL_CHECK=false; shift ;;
    --image|--work-dir|--output-dir) container_args+=("$1" "$2"); shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done
"${SCRIPT_DIR}/run_in_build_container.sh" "${container_args[@]}" -- bash -c '
  set -euo pipefail
  case "${ROS_DISTRO}" in
    melodic) packages=(xgc2_geometry_msgs) ;;
    noetic) packages=(xgc2_geometry_msgs cluttered_environment mockamap) ;;
    *) echo "unsupported ROS_DISTRO: ${ROS_DISTRO}" >&2; exit 1 ;;
  esac
  rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
  mkdir -p /workspace/work/src
  for package in "${packages[@]}"; do
    rsync -a --delete "/workspace/scene-generation/${package}/" "/workspace/work/src/${package}/"
  done
  cd /workspace/work
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  DESTDIR=/workspace/work/install-root catkin_make install \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/${ROS_DISTRO}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCATKIN_ENABLE_TESTING=OFF
  /workspace/scene-generation/.xgc2/scripts/package_debs.sh \
    --install-root /workspace/work/install-root --output-dir /workspace/out
'
