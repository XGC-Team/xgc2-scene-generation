#!/usr/bin/env bash
# Offline, original-tree validation. Compilation: 2 CPUs; execution: 1 CPU.
# Uses an already installed build image; no pull, clone, worktree or station.
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
work_dir="${1:-$(mktemp -d /tmp/xgc-scene-pr6-validation-XXXXXX)}"
image="ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.0"
base=f81594913414f1cdc15a59ab126d841cd6b00bc4
mkdir -p "$work_dir/base"
work_dir="$(cd "$work_dir" && pwd)"
git -C "$repo_root" archive "$base" xgc2_world_lidar | tar -x -C "$work_dir/base"
{
  git -C "$repo_root" rev-parse HEAD
  git -C "$repo_root" status --short
  docker image inspect "$image" --format '{{.Id}}'
} > "$work_dir/inputs.txt"
run_container() {
  local cpus="$1"
  shift
  docker run --rm -i --pull never --network none --cpus "$cpus" --entrypoint bash \
    --mount "type=bind,src=$repo_root,dst=/source,readonly" \
    --mount "type=bind,src=$work_dir,dst=/evidence" "$image" "$@"
}
run_container 2 -s > "$work_dir/build.log" 2>&1 <<'BUILD'
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
for variant in base candidate; do
  path=/source
  extra=-DPR6_INCREMENTAL
  if [ "$variant" = base ]; then path=/evidence/base; extra=; fi
  cmake -S "$path/xgc2_world_lidar" -B "/evidence/core-$variant" \
    -DCMAKE_DISABLE_FIND_PACKAGE_catkin=ON -DCMAKE_BUILD_TYPE=Release
  cmake --build "/evidence/core-$variant" -j2
  g++ -std=c++17 -O2 $extra -I"$path/xgc2_world_lidar/include" -I/usr/include/eigen3 \
    /source/xgc2_world_lidar/test/dump_observation_regression.cpp \
    "/evidence/core-$variant/libxgc2_world_lidar.a" -pthread -o "/evidence/dump-$variant"
done
source /opt/ros/noetic/setup.bash
mkdir -p /evidence/ros/src
for pkg in xgc2_geometry_msgs xgc2_scene_runtime xgc2_world_lidar; do
  ln -sfn "/source/$pkg" "/evidence/ros/src/$pkg"
done
cd /evidence/ros
catkin_make -DCMAKE_INSTALL_PREFIX=/evidence/installed -j2 -l2 \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=ON \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin_make -DCMAKE_INSTALL_PREFIX=/evidence/installed -j2 -l2 install
source /evidence/ros/devel/setup.bash
cmake -S /evidence/base/xgc2_world_lidar -B /evidence/base-ros \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF \
  -DCATKIN_DEVEL_PREFIX=/evidence/base-ros-devel
cmake --build /evidence/base-ros --target world_lidar_node -j2
BUILD
run_container 1 -s > "$work_dir/tests.log" 2>&1 <<'TESTS'
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1 ROS_IP=127.0.0.1 ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOME=/evidence/ros-home
unset ROS_HOSTNAME
for variant in base candidate; do
  (cd "/evidence/core-$variant" && ctest --output-on-failure)
  "/evidence/dump-$variant" > "/evidence/$variant-observations.bin"
done
cmp /evidence/base-observations.bin /evidence/candidate-observations.bin
sha256sum /evidence/*-observations.bin
source /evidence/ros/devel/setup.bash
cd /evidence/ros
catkin_make -DCMAKE_INSTALL_PREFIX=/evidence/installed -j1 -l1 \
  run_tests_xgc2_scene_runtime run_tests_xgc2_world_lidar
catkin_test_results --all build/test_results
rostest xgc2_world_lidar original_comparison.test \
  baseline_executable:=/evidence/base-ros-devel/lib/xgc2_world_lidar/world_lidar_node
# The installed executable, with installed message/package discovery, runs the
# same real ROS lifecycle tests. Test drivers alone remain in the source tree.
source /evidence/installed/setup.bash
export ROS_PACKAGE_PATH=/evidence/installed/share:/opt/ros/noetic/share
python3 - <<'PY'
import roslib.packages
import xgc2_geometry_msgs
import xgc2_scene_runtime
path = roslib.packages.find_node('xgc2_world_lidar', 'world_lidar_node')[0]
assert path.startswith('/evidence/installed/'), path
print('Installed ROS executable:', path)
for module in (xgc2_geometry_msgs, xgc2_scene_runtime):
    assert module.__file__.startswith('/evidence/installed/'), module.__file__
    print('Installed Python module:', module.__file__)
PY
python3 - <<'PY'
import subprocess
import sys
import time
import unittest

import rosgraph
import rospy

# Import test drivers only. Production package and executable discovery stays
# in the installed prefix and system ROS, without a source/devel fallback.
sys.path.insert(0, '/source/xgc2_world_lidar/test')
from cache_lifecycle_test import CacheLifecycle

with open('/evidence/installed-roscore.log', 'w') as log:
    master = subprocess.Popen(['roscore', '-p', '11311'], stdout=log, stderr=log)
    try:
        deadline = time.monotonic() + 10
        while True:
            assert master.poll() is None, 'isolated roscore exited'
            try:
                rosgraph.Master('/installed_validation').getPid()
                break
            except (OSError, rosgraph.MasterException):
                assert time.monotonic() < deadline, 'isolated roscore did not start'
                time.sleep(0.05)
        rospy.init_node('installed_cache_validation')
        suite = unittest.defaultTestLoader.loadTestsFromTestCase(CacheLifecycle)
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        assert result.wasSuccessful(), 'installed lifecycle checks failed'
    finally:
        rospy.signal_shutdown('installed validation finished')
        master.terminate()
        try:
            master.wait(timeout=5)
        except subprocess.TimeoutExpired:
            master.kill()
            master.wait()
PY
TESTS
printf 'PR6 validation passed; evidence: %s\n' "$work_dir"
