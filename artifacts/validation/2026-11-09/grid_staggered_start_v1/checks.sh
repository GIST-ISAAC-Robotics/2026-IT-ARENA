#!/usr/bin/env bash
# 사용: bash checks.sh   (실행 전 기하 사전 점검과 관련 회귀. 각 단계 실제 종료 코드를 기록한다)
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-09/grid_staggered_start_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" python3 -B "$A/geometry.py" > "$A/geometry.log" 2>&1
g=$?; printf '%s\n' "$g" > "$A/geometry.exit"
bash "$ENV" python3 -B -m pytest -q -p no:cacheprovider --tb=short tests/test_official_multi_actor_course.py tests/test_traffic_course_track.py tests/test_traffic_actors.py tests/test_official_traffic_grid.py tests/test_source_provenance_r0_plus_b3.py > "$A/regression.log" 2>&1
r=$?; printf '%s\n' "$r" > "$A/regression.exit"
echo "GEOMETRY_EXIT=$g REGRESSION_EXIT=$r"
[ "$g" -eq 0 ] && [ "$r" -eq 0 ]
