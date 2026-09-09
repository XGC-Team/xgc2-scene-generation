#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s "ros-${ROS_DISTRO}-xgc2-geometry-msgs" >/dev/null
test "$(rospack find xgc2_geometry_msgs)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_geometry_msgs"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_geometry_msgs/ConvexBodyArray.h"
rosmsg show xgc2_geometry_msgs/GeometryLibrary >/dev/null
rosmsg show xgc2_geometry_msgs/ConvexBodyArray >/dev/null
case "${ROS_DISTRO}" in
  melodic)
    python2 -c "from xgc2_geometry_msgs.msg import GeometryLibrary, ConvexBodyArray"
    echo "Installed Melodic message package check passed"
    exit 0
    ;;
  noetic) python3 -c "from xgc2_geometry_msgs.msg import GeometryLibrary, ConvexBodyArray" ;;
  *) echo "unsupported ROS_DISTRO: ${ROS_DISTRO}" >&2; exit 1 ;;
esac

dpkg -s ros-noetic-xgc2-scene-generation >/dev/null
dpkg -s ros-noetic-xgc2-cluttered-environment >/dev/null
dpkg -s ros-noetic-xgc2-geometry-msgs >/dev/null
dpkg -s ros-noetic-xgc2-mockamap >/dev/null
dpkg -s libxgc2-math-dev >/dev/null
math_version="$(dpkg-query -W -f='${Version}' libxgc2-math-dev)"
dpkg --compare-versions "${math_version}" ge '0.5.6-6~focal'

test "$(rospack find xgc2_geometry_msgs)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_geometry_msgs"
test "$(rospack find cluttered_environment)" = "/opt/ros/${ROS_DISTRO}/share/cluttered_environment"
test "$(rospack find mockamap)" = "/opt/ros/${ROS_DISTRO}/share/mockamap"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_geometry_msgs/ConvexBodyArray.h"
test -f "/usr/include/xgc2_math/geometry/math_helpers.h"
test -f "/usr/include/xgc2_math/geometry/collision/distance_gjk_query.h"
test -x "/opt/ros/${ROS_DISTRO}/lib/cluttered_environment/cluttered_environment_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/cluttered_environment/velocity_commander.py"
test -x "/opt/ros/${ROS_DISTRO}/lib/mockamap/mockamap_node"

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

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find "/opt/ros/${ROS_DISTRO}/lib/mockamap" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
