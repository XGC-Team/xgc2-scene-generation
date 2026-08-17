#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

WORK_DIR="${CPP_QUALITY_WORK_DIR:-${REPO_ROOT}/.work/cpp-quality}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
FORMAT_ONLY=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --format-only)
      FORMAT_ONLY=true
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "required command not found: $1" >&2
    exit 1
  fi
}

require_command clang-format
require_command clang-tidy
require_command rsync

mapfile -t cpp_files < <(
  find "${REPO_ROOT}/cluttered_environment" \
    \( -path '*/build/*' -o -path '*/devel/*' -o -path '*/install/*' -o -path '*/vendored/*' \) -prune \
    -o -type f \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.cc' -o -name '*.cxx' \) -print |
    sort
)

if [[ ${#cpp_files[@]} -eq 0 ]]; then
  echo "no C++ source files found under cluttered_environment" >&2
  exit 1
fi

echo "Checking clang-format on ${#cpp_files[@]} C++ files"
clang-format -n -Werror "${cpp_files[@]}"

if [[ "${FORMAT_ONLY}" == "true" ]]; then
  exit 0
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "missing /opt/ros/${ROS_DISTRO}/setup.bash; rerun with ROS ${ROS_DISTRO} installed" >&2
  exit 1
fi

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
require_command catkin_make

rm -rf "${WORK_DIR}/src" "${WORK_DIR}/build" "${WORK_DIR}/devel"
mkdir -p "${WORK_DIR}/src"
rsync -a --delete "${REPO_ROOT}/xgc2_geometry_msgs/" "${WORK_DIR}/src/xgc2_geometry_msgs/"
rsync -a --delete "${REPO_ROOT}/cluttered_environment/" "${WORK_DIR}/src/cluttered_environment/"
cp "${REPO_ROOT}/.clang-tidy" "${WORK_DIR}/src/.clang-tidy"

(
  cd "${WORK_DIR}"
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  catkin_make \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-O2 -g -DNDEBUG"
)

compile_db="${WORK_DIR}/build/compile_commands.json"
if [[ ! -f "${compile_db}" ]]; then
  echo "compile database was not generated: ${compile_db}" >&2
  exit 1
fi

mapfile -t tidy_files < <(
  find "${WORK_DIR}/src/cluttered_environment" \
    \( -path '*/build/*' -o -path '*/devel/*' -o -path '*/install/*' -o -path '*/vendored/*' \) -prune \
    -o -type f \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.cc' -o -name '*.cxx' \) -print |
    sort
)

echo "Checking clang-tidy on ${#tidy_files[@]} C++ files"
for file in "${tidy_files[@]}"; do
  clang-tidy \
    -p "${WORK_DIR}/build" \
    -header-filter="${WORK_DIR}/src/cluttered_environment/(include|src)/.*" \
    "$file"
done

echo "C++ quality check passed"
