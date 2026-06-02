#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-convex-geometry >/dev/null
dpkg -s ros-noetic-xgc2-convex-geometry-core >/dev/null
dpkg -s ros-noetic-xgc2-cluttered-environment >/dev/null

test "$(rospack find convex_geometry)" = "/opt/ros/${ROS_DISTRO}/share/convex_geometry"
test "$(rospack find cluttered_environment)" = "/opt/ros/${ROS_DISTRO}/share/cluttered_environment"
test -f "/opt/ros/${ROS_DISTRO}/include/convex_geometry/ConvexBodyArray.h"
test -f "/opt/ros/${ROS_DISTRO}/include/convex_geometry/geometry/math_helpers.h"
test -x "/opt/ros/${ROS_DISTRO}/lib/cluttered_environment/cluttered_environment_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/cluttered_environment/velocity_commander.py"

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find "/opt/ros/${ROS_DISTRO}/lib/cluttered_environment" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
