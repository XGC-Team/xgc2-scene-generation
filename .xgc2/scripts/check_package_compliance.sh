#!/usr/bin/env bash
# shellcheck disable=SC2016
set -euo pipefail
# The grep probes intentionally check literal packaging expressions.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

required_files=(
  ".xgc2/product.yml"
  "cluttered_environment/package.xml"
  "cluttered_environment/CMakeLists.txt"
  "xgc2_scene_runtime/package.xml"
  "xgc2_scene_runtime/CMakeLists.txt"
  "xgc2_scene_runtime/scripts/scene_node"
  "mockamap/package.xml"
  "mockamap/CMakeLists.txt"
  "xgc2_world_lidar/package.xml"
  "xgc2_world_lidar/CMakeLists.txt"
  ".xgc2/scripts/package_debs.sh"
  ".xgc2/scripts/check_installed_packages.sh"
  ".xgc2/scripts/run_in_build_container.sh"
  ".xgc2/scripts/install_published_products.sh"
  ".xgc2/scripts/test_package_debs.py"
)

for file in "${required_files[@]}"; do
  test -f "${file}" || {
    echo "missing required package file: ${file}" >&2
    exit 1
  }
done

grep -q '<name>cluttered_environment</name>' cluttered_environment/package.xml
grep -q '<name>mockamap</name>' mockamap/package.xml
grep -q '<name>xgc2_world_lidar</name>' xgc2_world_lidar/package.xml
grep -q '^project(xgc2_world_lidar)' xgc2_world_lidar/CMakeLists.txt
grep -q 'xgc2_world_lidar' .xgc2/product.yml
grep -Fq 'world_lidar_pkg="ros-noetic-xgc2-world-lidar"' .xgc2/scripts/package_debs.sh
grep -q '^project(cluttered_environment)' cluttered_environment/CMakeLists.txt
grep -q '^project(mockamap)' mockamap/CMakeLists.txt
grep -q 'find_package(xgc2_math REQUIRED CONFIG)' cluttered_environment/CMakeLists.txt
grep -q 'pcl_ros' mockamap/CMakeLists.txt
grep -q 'xgc2-scene-generation' .xgc2/product.yml
grep -q 'distribution: focal' .xgc2/product.yml
for workflow in .github/workflows/ci.yml .github/workflows/release.yml; do
  for cell in amd64-focal-noetic arm64-focal-noetic; do
    grep -Fq "name: ${cell}" "${workflow}"
  done
done
for script in .xgc2/scripts/*.sh; do bash -n "${script}"; done
grep -q 'cluttered_environment' .xgc2/product.yml
grep -q 'mockamap' .xgc2/product.yml
grep -q 'libxgc2-math-dev (>= 0.5.6-6~focal)' .xgc2/product.yml
grep -q 'ros-noetic-xgc2-scene-generation' .xgc2/scripts/package_debs.sh
grep -Fq 'msgs_pkg="ros-noetic-xgc2-geometry-msgs"' .xgc2/scripts/package_debs.sh
grep -Fq 'env_pkg="ros-noetic-xgc2-cluttered-environment"' .xgc2/scripts/package_debs.sh
grep -Fq 'mockamap_pkg="ros-noetic-xgc2-mockamap"' .xgc2/scripts/package_debs.sh
grep -Fq '${msgs_pkg} (>= 1.2.0-13)' .xgc2/scripts/package_debs.sh
grep -Fq '${env_pkg} (>= 1.1.4-12)' .xgc2/scripts/package_debs.sh
grep -Fq '${mockamap_pkg} (>= 1.1.4-12)' .xgc2/scripts/package_debs.sh
grep -Fq "dpkg --compare-versions" .xgc2/scripts/check_installed_packages.sh
grep -Fq "0.5.6-6~focal" .xgc2/scripts/check_installed_packages.sh
if grep -Eq '^[[:space:]]*continue-on-error:[[:space:]]*true' .github/workflows/ci.yml; then
  echo "CI quality/test jobs must fail closed" >&2
  exit 1
fi

if find . \
  \( -path './.git' -o -path './.work' -o -path './build' -o -path './devel' -o -path './install' -o -path './vendored' \) -prune \
  -o -type f \( -name 'package.xml' -o -name 'CMakeLists.txt' \) -print |
  grep -E '^\./(convex_geometry|convex_geometry_environment)/' >/dev/null; then
  echo "legacy package metadata still exists under old package directories" >&2
  exit 1
fi

test ! -e xgc2_geometry_msgs || { echo "message source belongs to xgc2-ros-msgs" >&2; exit 1; }
grep -Fq 'xgc2-ros-msgs: rebuild' .xgc2/product.yml

echo "Package compliance check passed"
