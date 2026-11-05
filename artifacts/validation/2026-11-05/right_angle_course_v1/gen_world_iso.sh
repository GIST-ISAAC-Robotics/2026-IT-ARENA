#!/usr/bin/env bash
# 고립 코너 변형 → iso_world/ 생성 후 격리 사본에 설치.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-05/right_angle_course_v1
/usr/bin/python3 -B "$A/gen_world_iso.py" src/arena_gazebo/worlds/it_arena_official "$A/iso_world" > "$A/gen_world_iso.log" 2>&1
code=$?
echo "GEN_EXIT=$code" >> "$A/gen_world_iso.log"
[ "$code" = 0 ] || exit "$code"
bash "$A/install_world.sh" iso_world
