#!/usr/bin/env bash
# 공식 팀 파생 월드(읽기 전용) → course_world/ 생성. 기존 course_world/가 있으면 생성기가 거절한다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-05/right_angle_course_v1
/usr/bin/python3 -B "$A/gen_world.py" src/arena_gazebo/worlds/it_arena_official "$A/course_world" > "$A/gen_world.log" 2>&1
code=$?
echo "GEN_EXIT=$code" >> "$A/gen_world.log"
exit $code
