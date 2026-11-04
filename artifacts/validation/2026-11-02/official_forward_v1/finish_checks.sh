#!/usr/bin/env bash
# 소스 계약 생성, 노면 사후 평가, 영향권 회귀. 시뮬레이터 없음.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_forward_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$A/prepare_contract.py" > "$A/finish.log" 2>&1
echo "CONTRACT=$?" > "$A/finish.exit"
for n in a1 a2 b1 b2; do
  python3 -B scripts/surface_contact_eval.py "$A/$n" --output "$A/${n}_surface.json" >> "$A/finish.log" 2>&1
  echo "SURFACE_$n=$?" >> "$A/finish.exit"
done
python3 -B -m pytest -q -p no:cacheprovider --tb=line \
  tests/test_observed_motion_gates.py tests/test_traffic_merge.py tests/test_g1_traffic_eval.py tests/test_race_traffic_course.py \
  tests/test_rolling_lead_course.py tests/test_official_signal_control.py tests/test_source_provenance_r0_plus_b3.py \
  tests/test_official_track.py tests/test_official_start_region.py tests/test_official_start_local.py tests/test_g1_side_occlusion.py \
  tests/test_traffic_actor_integration.py tests/test_runtime_world_options.py tests/test_prelap_signal_eval.py tests/test_continuous_prefix.py \
  > "$A/tests_final.log" 2>&1
echo "TESTS=$?" >> "$A/finish.exit"
