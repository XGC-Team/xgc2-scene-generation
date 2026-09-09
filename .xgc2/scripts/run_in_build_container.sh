#!/usr/bin/env bash
# shellcheck disable=SC2016,SC1004
set -euo pipefail
# Literal scripts below are expanded by Bash inside the build container.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DOCKER_IMAGE="${DOCKER_IMAGE:-ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.0}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) DOCKER_IMAGE="$2"; shift 2 ;;
    --work-dir) WORK_DIR="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --) shift; break ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done
[[ $# -gt 0 ]] || { echo "a container command is required" >&2; exit 1; }
mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"
WORK_DIR="$(cd "${WORK_DIR}" && pwd)"
OUTPUT_DIR="$(cd "${OUTPUT_DIR}" && pwd)"
docker pull "${DOCKER_IMAGE}"
container_id=""
cleanup() {
  if [[ -n "${container_id}" ]]; then
    docker rm -f "${container_id}" >/dev/null
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# All workspace writes use the caller UID. Only the published-package install
# and final dpkg installation execute as root, inside this disposable container.
container_id="$(docker run --detach --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e DEBIAN_FRONTEND=noninteractive \
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
  -e PACKAGE_VERSION="${PACKAGE_VERSION:-}" \
  -v "${REPO_ROOT}:/workspace/scene-generation:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" sleep infinity)"
docker exec "${container_id}" bash -c '
  set -euo pipefail
  : "${ROS_DISTRO:?ROS_DISTRO must be set in the image}"
  for package in build-essential cmake dpkg-dev fakeroot file git rsync \
    "ros-${ROS_DISTRO}-message-generation" "ros-${ROS_DISTRO}-message-runtime" \
    "ros-${ROS_DISTRO}-geometry-msgs" "ros-${ROS_DISTRO}-std-msgs" \
    "ros-${ROS_DISTRO}-rosmsg" "ros-${ROS_DISTRO}-rospack"; do
    dpkg -s "${package}" >/dev/null || {
      echo "build image lacks ${package}; repair xgc2-images, not the product" >&2
      exit 1
    }
  done
'
docker exec --user 0 "${container_id}" \
  /workspace/scene-generation/.xgc2/scripts/install_published_products.sh
docker exec "${container_id}" "$@"
if [[ "${INSTALL_CHECK:-false}" == true ]]; then
  docker exec --user 0 "${container_id}" bash -c '
    set -euo pipefail
    apt-get install -y --no-install-recommends /workspace/out/*.deb
    /workspace/scene-generation/.xgc2/scripts/check_installed_packages.sh
  '
fi
