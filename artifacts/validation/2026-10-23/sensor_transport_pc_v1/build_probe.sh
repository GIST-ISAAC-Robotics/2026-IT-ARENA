#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
evidence=artifacts/validation/2026-10-23/sensor_transport_pc_v1
test ! -e "$evidence/build_exit.txt"
finish() { result=$?; trap - EXIT; printf '%s\n' "$result" > "$evidence/build_exit.txt"; exit "$result"; }
trap finish EXIT
mkdir -p build/sensor_transport_pc_v1
test ! -e build/sensor_transport_pc_v1/libprobe.so
sha256sum scripts/dds_transport_probe.cpp > "$evidence/source_sha256.txt"
g++ -std=c++17 -shared -fPIC -O2 scripts/dds_transport_probe.cpp \
 -I/opt/ros/jazzy/include/fastrtps -I/opt/ros/jazzy/include/fastcdr \
 -L/opt/ros/jazzy/lib -Wl,-rpath,/opt/ros/jazzy/lib -lfastrtps \
 -o build/sensor_transport_pc_v1/libprobe.so > "$evidence/build.log" 2>&1
sha256sum build/sensor_transport_pc_v1/libprobe.so > "$evidence/library_sha256.txt"
dpkg-query -W ros-jazzy-rmw-fastrtps-cpp ros-jazzy-fastrtps > "$evidence/package_versions.txt"
