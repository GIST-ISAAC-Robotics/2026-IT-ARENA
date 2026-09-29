#!/usr/bin/env bash
# 엔코더 발행/수신 계측과 executor 교차 비교. 기존 결과는 보존한다.
set -eo pipefail
cd /home/q/it_arena_a3_20260928
source /opt/ros/humble/setup.bash
colcon build --packages-select arena_autonomy arena_vehicle_interface --event-handlers console_direct+
source install/setup.bash
python3 -m pytest src/arena_vehicle_interface/test/test_lifecycle.py \
  tests/test_node_lifecycle_modes.py tests/test_wheel_delivery_analysis.py tests/test_replay_continuity.py \
  tests/test_vision_process.py tests/test_signal_mask.py tests/test_replay_observer.py \
  tests/test_bag_contract.py tests/test_bag_reader_compat.py tests/test_timing_probe.py tests/test_queued_log.py \
  tests/test_lidar_motion.py tests/test_lidar_motion_ros.py tests/test_lidar_safety.py \
  tests/test_local_path_range_noise.py -q -k 'not overshot_corner'
recording=artifacts/validation/2026-09-24/record_replay/capture_v2/sensor_recording
replays=()
for case_name in legacy_a retained_a retained_b legacy_b; do
    mode=${case_name%_*}
    target=artifacts/jetson_a3/wheel_dispatch_v1_${case_name}
    python3 scripts/run_jetson_replay_trial.py "$recording" --output "$target" \
      --rate 1 --executor-mode "$mode" --trace-wheels-publication
    python3 scripts/audit_replay_delivery.py "$recording" "$target/replay"
    replays+=("$target/replay")
done
python3 scripts/analyze_wheel_delivery.py "${replays[@]}" \
  --output artifacts/jetson_a3/wheel_dispatch_comparison_v1.json --recording "$recording"
echo WHEEL_DISPATCH_TRIALS_COMPLETE
