#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

WORK_DIR="${ROS_PACKAGE_CHECK_WORK_DIR:-${REPO_ROOT}/.work/package-tests}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "missing /opt/ros/${ROS_DISTRO}/setup.bash; rerun with ROS ${ROS_DISTRO} installed" >&2
  exit 1
fi

command -v catkin_make >/dev/null 2>&1 || {
  echo "required command not found: catkin_make" >&2
  exit 1
}
command -v catkin_test_results >/dev/null 2>&1 || {
  echo "required command not found: catkin_test_results" >&2
  exit 1
}
command -v rsync >/dev/null 2>&1 || {
  echo "required command not found: rsync" >&2
  exit 1
}

rm -rf "${WORK_DIR}/src" "${WORK_DIR}/build" "${WORK_DIR}/devel"
mkdir -p "${WORK_DIR}/src"
rsync -a --delete "${REPO_ROOT}/xgc2_geometry_msgs/" "${WORK_DIR}/src/xgc2_geometry_msgs/"
rsync -a --delete "${REPO_ROOT}/cluttered_environment/" "${WORK_DIR}/src/cluttered_environment/"
rsync -a --delete "${REPO_ROOT}/mockamap/" "${WORK_DIR}/src/mockamap/"

(
  cd "${WORK_DIR}"
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  catkin_make \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCATKIN_ENABLE_TESTING=ON
  catkin_make run_tests
  catkin_test_results
)

echo "ROS package build and test check passed"
