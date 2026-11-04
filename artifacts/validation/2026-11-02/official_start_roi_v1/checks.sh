#!/usr/bin/env bash
# 검출 영역 영향권 회귀. 시뮬레이터 없음. 인자: 로그 이름.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_start_roi_v1
NAME=${1:-tests_focus}
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest -q -p no:cacheprovider --tb=short \
  tests/test_official_start_region.py tests/test_official_start_local.py tests/test_official_start_signal.py \
  tests/test_official_vision_pipeline.py tests/test_b0183_image_handoff.py tests/test_official_start_ros_fixture.py \
  tests/test_prelap_signal_eval.py tests/test_official_signal_control.py tests/test_official_track.py \
  tests/test_source_provenance_r0_plus_b3.py tests/test_runtime_world_options.py tests/test_start_signal_render_fixture.py \
  > "$A/$NAME.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/$NAME.exit"
exit "$code"
