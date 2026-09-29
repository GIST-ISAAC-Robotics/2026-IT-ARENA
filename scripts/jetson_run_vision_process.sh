#!/usr/bin/env bash
# 센서/보호 조건을 유지한 영상 프로세스 분리 시험. 기존 결과는 덮어쓰지 않는다.
set -eo pipefail
cd /home/q/it_arena_a3_20260928
source /opt/ros/humble/setup.bash
colcon build --packages-select arena_autonomy arena_vehicle_interface --event-handlers console_direct+
source install/setup.bash
python3 -m pytest tests/test_vision_process.py tests/test_signal_mask.py tests/test_replay_observer.py \
  tests/test_bag_contract.py tests/test_bag_reader_compat.py tests/test_timing_probe.py tests/test_queued_log.py \
  tests/test_lidar_motion.py tests/test_lidar_motion_ros.py tests/test_lidar_safety.py \
  tests/test_local_path_range_noise.py -q -k 'not overshot_corner'
recording=artifacts/validation/2026-09-24/record_replay/capture_v2/sensor_recording
python3 scripts/check_vision_worker_equivalence.py "$recording" \
  --output artifacts/jetson_a3/vision_process_equivalence_v1.json
for name in vision_process_v1_a vision_process_v1_b; do
    result=0
    python3 scripts/run_jetson_replay_trial.py "$recording" --output "artifacts/jetson_a3/$name" --rate 1 || result=$?
    echo "TRIAL_EXIT $name $result"
    test -f "artifacts/jetson_a3/$name/replay/report.json"
    python3 scripts/audit_replay_delivery.py "$recording" "artifacts/jetson_a3/$name/replay"
done
echo VISION_PROCESS_TRIALS_COMPLETE
