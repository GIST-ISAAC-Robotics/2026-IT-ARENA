#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/grid_pack_newcourse_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" python3 -B -m pytest -q tests/test_official_multi_actor_course.py tests/test_traffic_course_track.py tests/test_traffic_actors.py tests/test_official_traffic_grid.py tests/test_source_provenance_r0_plus_b3.py > "$A/regression.log" 2>&1
r=$?;printf '%s\n' "$r" > "$A/regression.exit";exit "$r"
