#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
VERSION="${PACKAGE_VERSION:-1.1.4-1}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}"/*.deb

copy_path() {
  local src="$1"
  local dst_root="$2"
  if [[ -e "${src}" ]]; then
    mkdir -p "${dst_root}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${dst_root}${src#${INSTALL_ROOT}}"
  fi
}

write_control() {
  local pkg_root="$1"
  local package="$2"
  local depends="$3"
  local description="$4"

  mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${package}"
  cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${package}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: ${depends}
Description: ${description}
EOF
  printf '%s package\n' "${package}" > "${pkg_root}/usr/share/doc/${package}/README"
  chmod 0755 "${pkg_root}/DEBIAN"
}

copy_ros_package_paths() {
  local ros_pkg="$1"
  local dst_root="$2"

  copy_path "${PREFIX_ROOT}/share/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/include/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/lib/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/lib/python3/dist-packages/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/gennodejs/ros/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/common-lisp/ros/${ros_pkg}" "${dst_root}"
  copy_path "${PREFIX_ROOT}/share/roseus/ros/${ros_pkg}" "${dst_root}"
}

build_ros_package_deb() {
  local package="$1"
  local ros_pkg="$2"
  local depends="$3"
  local description="$4"

  local pkg_root="${BUILD_DIR}/${package}"
  rm -rf "${pkg_root}"
  mkdir -p "${pkg_root}"

  copy_ros_package_paths "${ros_pkg}" "${pkg_root}"
  write_control "${pkg_root}" "${package}" "${depends}" "${description}"
  fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${package}_${VERSION}_${ARCH}.deb" >/dev/null
}

msgs_pkg="ros-noetic-xgc2-geometry-msgs"
env_pkg="ros-noetic-xgc2-cluttered-environment"
mockamap_pkg="ros-noetic-xgc2-mockamap"
meta_pkg="ros-noetic-xgc2-scene-generation"

build_ros_package_deb \
  "${msgs_pkg}" \
  "xgc2_geometry_msgs" \
  "ros-noetic-message-runtime, ros-noetic-geometry-msgs, ros-noetic-std-msgs" \
  "XGC2 convex geometry template and obstacle instance messages"

build_ros_package_deb \
  "${env_pkg}" \
  "cluttered_environment" \
  "libxgc2-math-dev (>= 0.5.5-1), ${msgs_pkg} (= ${VERSION}), ros-noetic-roscpp, ros-noetic-rospy, ros-noetic-geometry-msgs, ros-noetic-std-msgs, ros-noetic-visualization-msgs, ros-noetic-tf2, ros-noetic-tf2-geometry-msgs, ros-noetic-tf2-ros, python3-yaml" \
  "XGC2 cluttered simulation environment manager"

build_ros_package_deb \
  "${mockamap_pkg}" \
  "mockamap" \
  "ros-noetic-roscpp, ros-noetic-sensor-msgs, ros-noetic-pcl-ros, ros-noetic-pcl-conversions, ros-noetic-rviz" \
  "XGC2 procedural point-cloud map generator"

meta_root="${BUILD_DIR}/${meta_pkg}"
rm -rf "${meta_root}"
mkdir -p "${meta_root}"
write_control \
  "${meta_root}" \
  "${meta_pkg}" \
  "${msgs_pkg} (= ${VERSION}), ${env_pkg} (= ${VERSION}), ${mockamap_pkg} (= ${VERSION})" \
  "XGC2 scene generation package set"
fakeroot dpkg-deb --build "${meta_root}" "${OUTPUT_DIR}/${meta_pkg}_${VERSION}_${ARCH}.deb" >/dev/null

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
