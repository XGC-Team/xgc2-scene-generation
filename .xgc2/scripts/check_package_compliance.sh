#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

required_files=(
  ".xgc2/product.yml"
  "xgc2_geometry_msgs/package.xml"
  "xgc2_geometry_msgs/CMakeLists.txt"
  "cluttered_environment/package.xml"
  "cluttered_environment/CMakeLists.txt"
  "mockamap/package.xml"
  "mockamap/CMakeLists.txt"
  ".xgc2/scripts/package_debs.sh"
  ".xgc2/scripts/check_installed_packages.sh"
)

for file in "${required_files[@]}"; do
  test -f "${file}" || {
    echo "missing required package file: ${file}" >&2
    exit 1
  }
done

grep -q '<name>xgc2_geometry_msgs</name>' xgc2_geometry_msgs/package.xml
grep -q '<name>cluttered_environment</name>' cluttered_environment/package.xml
grep -q '<name>mockamap</name>' mockamap/package.xml
grep -q '^project(xgc2_geometry_msgs)' xgc2_geometry_msgs/CMakeLists.txt
grep -q '^project(cluttered_environment)' cluttered_environment/CMakeLists.txt
grep -q '^project(mockamap)' mockamap/CMakeLists.txt
grep -q 'find_package(xgc2_geometry REQUIRED CONFIG)' cluttered_environment/CMakeLists.txt
grep -q 'pcl_ros' mockamap/CMakeLists.txt
grep -q 'xgc2-scene-generation' .xgc2/product.yml
grep -q 'xgc2_geometry_msgs' .xgc2/product.yml
grep -q 'cluttered_environment' .xgc2/product.yml
grep -q 'mockamap' .xgc2/product.yml
grep -q 'libxgc2-geometry-dev' .xgc2/product.yml
grep -q 'ros-noetic-xgc2-scene-generation' .xgc2/scripts/package_debs.sh
grep -q 'ros-noetic-xgc2-geometry-msgs' .xgc2/scripts/package_debs.sh
grep -q 'ros-noetic-xgc2-cluttered-environment' .xgc2/scripts/package_debs.sh
grep -q 'ros-noetic-xgc2-mockamap' .xgc2/scripts/package_debs.sh
grep -q 'libxgc2-geometry-dev' .xgc2/scripts/check_installed_packages.sh

if find . \
  \( -path './.git' -o -path './.work' -o -path './build' -o -path './devel' -o -path './install' -o -path './vendored' \) -prune \
  -o -type f \( -name 'package.xml' -o -name 'CMakeLists.txt' \) -print |
  grep -E '^\./(convex_geometry|convex_geometry_environment)/' >/dev/null; then
  echo "legacy package metadata still exists under old package directories" >&2
  exit 1
fi

echo "Package compliance check passed"
