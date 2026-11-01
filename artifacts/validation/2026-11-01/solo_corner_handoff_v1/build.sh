#!/usr/bin/env bash
# 등록 전 빌드. 파이썬 패키지의 설치본을 현재 소스와 맞춘다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-01/solo_corner_handoff_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
colcon build --packages-select arena_autonomy arena_bringup --symlink-install > "$A/build.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/build.exit"
exit "$code"
