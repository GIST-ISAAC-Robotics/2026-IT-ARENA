#!/usr/bin/env bash
# JetPack/OS/전력 설정을 유지하며 Humble 재생 시험 의존성만 설치한다.
set -euo pipefail
test "$(id -u)" = 0
. /etc/os-release
test "$VERSION_CODENAME" = jammy
test "$(dpkg --print-architecture)" = arm64
task_tmp=$(mktemp -d /tmp/it-arena-ros-deps.XXXXXX)
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
    curl --fail --location --retry 2 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest -o "$task_tmp/release.json"
    release=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["tag_name"])' "$task_tmp/release.json")
    [[ "$release" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]
    curl --fail --location --retry 2 "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$release/ros2-apt-source_${release}.jammy_all.deb" -o "$task_tmp/ros2-apt-source.deb"
    sha256sum "$task_tmp/ros2-apt-source.deb"
    dpkg -i "$task_tmp/ros2-apt-source.deb"
fi
apt-get update
packages=(ros-humble-ros-base ros-humble-ackermann-msgs ros-humble-rosbag2-py
          ros-humble-rosbag2-storage-mcap python3-yaml python3-numpy
          python3-pip python3-venv python3-colcon-common-extensions python3-pytest)
apt-get -s --no-remove install "${packages[@]}" | tee "$task_tmp/apt-plan.txt"
if grep -q '^Remv ' "$task_tmp/apt-plan.txt"; then
    echo 'Package removal proposed: stop for manual review' >&2
    exit 2
fi
DEBIAN_FRONTEND=noninteractive apt-get -y --no-remove install "${packages[@]}"
echo 'HUMBLE_REPLAY_DEPENDENCIES_INSTALLED'
