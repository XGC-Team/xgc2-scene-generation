#!/usr/bin/env bash
set -euo pipefail

# Only published product dependencies may be installed here. The versioned
# build image owns the compiler, official ROS packages, and Python toolchain.
: "${ROS_DISTRO:?ROS_DISTRO must be set in the build image}"
case "${ROS_DISTRO}" in
  melodic) exit 0 ;; # The Melodic install set contains only message definitions.
  noetic) suite=focal ;;
  *) echo "unsupported ROS_DISTRO: ${ROS_DISTRO}" >&2; exit 1 ;;
esac

install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg \
  -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
fingerprints="$(gpg --batch --show-keys --with-colons /etc/apt/keyrings/xgc2-archive-keyring.gpg \
  | awk -F: '$1 == "fpr" {print $10}')"
test "${fingerprints}" = 2A8E11B36F56D307ADF626D85E5FDC30979EA43F
chmod 0644 /etc/apt/keyrings/xgc2-archive-keyring.gpg
printf 'deb [signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] %s %s main\n' \
  https://xgc2.apt.xiaokang.ink "${suite}" > /etc/apt/sources.list.d/xgc2.list
if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
  printf 'deb [signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] %s %s main\n' \
    "${XGC2_APT_OVERLAY_URL%/}" "${suite}" > /etc/apt/sources.list.d/00-xgc2-release-train.list
fi
apt-get update
apt-get install -y --no-install-recommends libxgc2-math-dev
