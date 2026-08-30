#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script as root: sudo bash scripts/setup_wsl.sh" >&2
  exit 1
fi

if [[ ! -r /etc/os-release ]]; then
  echo "Cannot identify the operating system." >&2
  exit 1
fi

source /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_ID}" != "24.04" ]]; then
  echo "This setup is pinned to Ubuntu 24.04; found ${ID} ${VERSION_ID}." >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y locales software-properties-common curl ca-certificates gnupg lsb-release git
locale-gen en_US en_US.UTF-8
update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
add-apt-repository -y universe

tmp_dir="$(mktemp -d)"
trap 'rm -rf "${tmp_dir}"' EXIT

ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p' | head -n 1)"
if [[ -z "${ros_apt_version}" ]]; then
  echo "Could not resolve the current ros2-apt-source release." >&2
  exit 1
fi

curl -fsSL -o "${tmp_dir}/ros2-apt-source.deb" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.noble_all.deb"
dpkg -i "${tmp_dir}/ros2-apt-source.deb"

apt-get update
apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-ackermann-msgs \
  ros-jazzy-cv-bridge \
  ros-jazzy-xacro \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-opencv \
  python3-matplotlib \
  python3-shapely \
  python3-rosdep \
  python3-vcstool

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi

echo "System dependencies installed. Run scripts/configure_wsl_user.sh as the normal WSL user next."
