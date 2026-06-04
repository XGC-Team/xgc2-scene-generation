#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -v "${REPO_ROOT}:/workspace/scene-generation:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      curl \
      dpkg-dev \
      fakeroot \
      file \
      git \
      libeigen3-dev \
      python3-yaml \
      rsync \
      ros-noetic-geometry-msgs \
      ros-noetic-message-generation \
      ros-noetic-message-runtime \
      ros-noetic-pcl-conversions \
      ros-noetic-pcl-ros \
      ros-noetic-roscpp \
      ros-noetic-rospy \
      ros-noetic-rviz \
      ros-noetic-sensor-msgs \
      ros-noetic-rospack \
      ros-noetic-std-msgs \
      ros-noetic-tf2 \
      ros-noetic-tf2-geometry-msgs \
      ros-noetic-tf2-ros \
      ros-noetic-visualization-msgs

    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg \
      -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
    chmod 0644 /etc/apt/keyrings/xgc2-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] https://xgc2.apt.xiaokang.ink focal main" \
      > /etc/apt/sources.list.d/xgc2.list
    apt-get update
    apt-get install -y --no-install-recommends libxgc2-geometry-dev

    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src
    rsync -a --delete /workspace/scene-generation/xgc2_geometry_msgs/ /workspace/work/src/xgc2_geometry_msgs/
    rsync -a --delete /workspace/scene-generation/cluttered_environment/ /workspace/work/src/cluttered_environment/
    rsync -a --delete /workspace/scene-generation/mockamap/ /workspace/work/src/mockamap/

    cd /workspace/work
    source /opt/ros/noetic/setup.bash

    catkin_make \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    DESTDIR=/workspace/work/install-root catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_BUILD_TYPE=Release \
      -DCATKIN_ENABLE_TESTING=OFF \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    /workspace/scene-generation/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y \
        /workspace/out/ros-noetic-xgc2-scene-generation_*.deb \
        /workspace/out/ros-noetic-xgc2-geometry-msgs_*.deb \
        /workspace/out/ros-noetic-xgc2-cluttered-environment_*.deb \
        /workspace/out/ros-noetic-xgc2-mockamap_*.deb
      /workspace/scene-generation/.xgc2/scripts/check_installed_packages.sh
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
