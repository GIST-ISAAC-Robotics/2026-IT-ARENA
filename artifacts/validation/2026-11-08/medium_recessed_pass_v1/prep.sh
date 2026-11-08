#!/usr/bin/env bash
# 등록 전 1회: 정적 SDF 대조, gz sdf -k, 대상 외형 단위검사(기존 2파일), r15 저장 판단 재생.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/medium_recessed_pass_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
bash "$ENV" /usr/bin/python3 -B "$A/static_sdf_check.py" > "$A/static_sdf_check.log" 2>&1; s=$?
gz sdf -k "$A/static_recessed_current_code.sdf" > "$A/gz_sdf_check.log" 2>&1; g=$?
bash "$ENV" /usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_traffic_actor_outline.py tests/test_actor_outline_returns.py > "$A/outline_tests.log" 2>&1; t=$?
bash "$ENV" /usr/bin/python3 -B "$A/replay_r15.py" > "$A/replay_r15.log" 2>&1; r=$?
echo "STATIC=$s GZ_SDF=$g TESTS=$t REPLAY=$r" > "$A/prep.exit"
