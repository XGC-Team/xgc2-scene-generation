#!/usr/bin/env bash
# shellcheck disable=SC2016
set -euo pipefail
# The grep probes intentionally check literal packaging expressions.

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

grep -q '<name>xgc2_geometry_msgs</name>' xgc2_geometry_msgs/package.xml
grep -q '<name>cluttered_environment</name>' cluttered_environment/package.xml
grep -q '<name>mockamap</name>' mockamap/package.xml
grep -q '^project(xgc2_geometry_msgs)' xgc2_geometry_msgs/CMakeLists.txt
grep -q '^project(cluttered_environment)' cluttered_environment/CMakeLists.txt
grep -q '^project(mockamap)' mockamap/CMakeLists.txt
grep -q 'find_package(xgc2_math REQUIRED CONFIG)' cluttered_environment/CMakeLists.txt
grep -q 'pcl_ros' mockamap/CMakeLists.txt
grep -q 'xgc2-scene-generation' .xgc2/product.yml
grep -q 'distribution: bionic,focal' .xgc2/product.yml
grep -Fq 'ros-melodic-xgc2-geometry-msgs: [bionic]' .xgc2/product.yml
grep -Fq 'ros-noetic-xgc2-geometry-msgs: [focal]' .xgc2/product.yml
for workflow in .github/workflows/ci.yml .github/workflows/release.yml; do
  for cell in amd64-focal-noetic arm64-focal-noetic amd64-bionic-melodic arm64-bionic-melodic; do
    grep -Fq "name: ${cell}" "${workflow}"
  done
done
for script in .xgc2/scripts/*.sh; do bash -n "${script}"; done
grep -q 'xgc2_geometry_msgs' .xgc2/product.yml
grep -q 'cluttered_environment' .xgc2/product.yml
grep -q 'mockamap' .xgc2/product.yml
grep -q 'libxgc2-math-dev (>= 0.5.6-6~focal)' .xgc2/product.yml
grep -q 'ros-noetic-xgc2-scene-generation' .xgc2/scripts/package_debs.sh
grep -Fq 'msgs_pkg="ros-${ROS_DISTRO}-xgc2-geometry-msgs"' .xgc2/scripts/package_debs.sh
grep -Fq 'env_pkg="ros-noetic-xgc2-cluttered-environment"' .xgc2/scripts/package_debs.sh
grep -Fq 'mockamap_pkg="ros-noetic-xgc2-mockamap"' .xgc2/scripts/package_debs.sh
grep -Fq '${msgs_pkg} (>= 1.1.4-12)' .xgc2/scripts/package_debs.sh
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

echo "Package compliance check passed"
