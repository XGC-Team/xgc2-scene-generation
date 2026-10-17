#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s "ros-${ROS_DISTRO}-xgc2-geometry-msgs" >/dev/null
test "$(rospack find xgc2_geometry_msgs)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_geometry_msgs"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_geometry_msgs/SceneGeometry.h"
rosmsg show xgc2_geometry_msgs/SceneSnapshot >/dev/null
[[ "${ROS_DISTRO}" == noetic ]] || { echo "unsupported ROS_DISTRO: ${ROS_DISTRO}" >&2; exit 1; }
python3 -c "from xgc2_geometry_msgs.msg import SceneSnapshot, SceneGeometry; assert SceneGeometry.__slots__ == ['type', 'parameters']; assert SceneGeometry._slot_types == ['uint32', 'uint8[]']"
geometry_version="$(dpkg-query -W -f='${Version}' ros-noetic-xgc2-geometry-msgs)"
dpkg --compare-versions "${geometry_version}" ge '1.5.0-4'

dpkg -s ros-noetic-xgc2-scene-generation >/dev/null
dpkg -s ros-noetic-xgc2-scene-runtime >/dev/null
test ! -e "/opt/ros/noetic/lib/xgc2_scene_runtime/scene_node"
test ! -e "/opt/ros/noetic/lib/xgc2_scene_runtime/scene_prepare.py"
test ! -e "/opt/ros/noetic/share/xgc2_scene_runtime/launch/scene.launch"
python3 - <<'PY_CHECK'
from xgc2_math.geometry import encode_geometry, decode_geometry
assert decode_geometry(*encode_geometry({"type": "ring", "radii": [3, 1], "tubeRadius": .8})) == {"type": "ring", "radii": [3, 1], "tubeRadius": .8}
from xgc2_scene_runtime.store import SceneStore
PY_CHECK
dpkg -s ros-noetic-xgc2-cluttered-environment >/dev/null
dpkg -s ros-noetic-xgc2-geometry-msgs >/dev/null
dpkg -s ros-noetic-xgc2-mockamap >/dev/null
dpkg -s ros-noetic-xgc2-world-lidar >/dev/null
test "$(rospack find xgc2_world_lidar)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_world_lidar"
test ! -e "/opt/ros/${ROS_DISTRO}/lib/xgc2_world_lidar/world_lidar_node"
test ! -e "/opt/ros/${ROS_DISTRO}/share/xgc2_world_lidar/launch/world_lidar.launch"
test -f "/opt/ros/${ROS_DISTRO}/lib/libxgc2_world_lidar.a"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_world_lidar/world_lidar.h"
dpkg -s libxgc2-math-dev >/dev/null
math_version="$(dpkg-query -W -f='${Version}' libxgc2-math-dev)"
dpkg --compare-versions "${math_version}" ge '0.6.0-6~focal'

test "$(rospack find xgc2_geometry_msgs)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_geometry_msgs"
test "$(rospack find cluttered_environment)" = "/opt/ros/${ROS_DISTRO}/share/cluttered_environment"
test "$(rospack find mockamap)" = "/opt/ros/${ROS_DISTRO}/share/mockamap"
test -f "/opt/ros/${ROS_DISTRO}/include/xgc2_geometry_msgs/SceneSnapshot.h"
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
