#!/usr/bin/env bash
# 소스 확정 후 계약 생성, 교통 장면 정적 대조, 실패했던 항목과 영향권 재검사.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_facility_v1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
{
  python3 -B "$A/prepare_contract.py" &&
  python3 -B "$A/traffic_scene_audit.py" &&
  python3 -B -m pytest -q -p no:cacheprovider --tb=line $(cat "$A/failed_nodes.txt" | sed 's/\[.*$//' | sort -u | tr '\n' ' ') \
    tests/test_official_track.py tests/test_official_signal_control.py tests/test_runtime_world_options.py tests/test_g1_side_occlusion.py
} > "$A/checks2.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/checks2.exit"
exit "$code"
