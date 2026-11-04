#!/usr/bin/env bash
# 월드·신호 영향권 회귀. 판정선은 바꾸지 않는다. 시뮬레이터를 띄우지 않는다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
A=artifacts/validation/2026-11-02/official_facility_v1
NAME=${1:-tests_wide}
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest -q -p no:cacheprovider \
  tests/test_official_track.py tests/test_official_signal_control.py tests/test_track_variants.py \
  tests/test_experimental_facilities.py tests/test_runtime_world_options.py tests/test_course_marker_visibility.py \
  tests/test_evaluation_startup.py tests/test_g1_traffic_eval.py tests/test_gantry_marker_face.py tests/test_local_pursuit.py \
  tests/test_physics_step_option.py tests/test_surface_contact_eval.py tests/test_wheel_contact_probe.py tests/test_grid_start.py \
  tests/test_start_signal_geometry.py tests/test_start_signal_render_fixture.py tests/test_prelap_signal_eval.py \
  tests/test_race_traffic_course.py tests/test_rolling_lead_course.py tests/test_basic_autonomy.py \
  tests/test_official_start_local.py tests/test_official_start_signal.py tests/test_official_vision_pipeline.py \
  tests/test_official_start_ros_fixture.py tests/test_grid_camera_pitch.py \
  > "$A/$NAME.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > "$A/$NAME.exit"
exit "$code"
