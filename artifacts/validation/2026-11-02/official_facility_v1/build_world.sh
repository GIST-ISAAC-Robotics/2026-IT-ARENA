#!/usr/bin/env bash
# 공식 v2026.11.01 ZIP에서 팀 실행 월드 재생성 + 해시 검사. 원본 ZIP은 읽기만 한다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_facility_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
{ python3 -B scripts/build_official_track.py > "$A/build_world.json" && python3 -B scripts/build_official_track.py --check; } > "$A/build_world.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/build_world.exit"
exit "$code"
