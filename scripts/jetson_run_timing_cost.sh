#!/usr/bin/env bash
# 한정된 ABBA 계측 비교. 재시도/설정 변경/구동부 실행 없음.
set -eo pipefail
cd /home/q/it_arena_a3_20260928
source /opt/ros/humble/setup.bash
colcon build --packages-select arena_vehicle_interface arena_autonomy --executor sequential
source install/setup.bash
python3 - <<'PY'
import hashlib, inspect, pathlib
from arena_vehicle_interface import timing_probe
source = pathlib.Path('src/arena_vehicle_interface/arena_vehicle_interface/timing_probe.py')
installed = pathlib.Path(inspect.getfile(timing_probe))
assert hashlib.sha256(source.read_bytes()).digest() == hashlib.sha256(installed.read_bytes()).digest()
print('TIMING_PROBE_INSTALLED_SOURCE_MATCH', str(installed))
PY
python3 -m pytest -q tests/test_timing_probe.py tests/test_replay_timing.py \
  tests/test_replay_observer.py tests/test_replay_continuity.py tests/test_bag_contract.py \
  tests/test_replay_transport.py tests/test_imu_delivery_analysis.py tests/test_wheel_delivery_analysis.py
recording=artifacts/validation/2026-09-29/imu_long_capture/capture_v3/sensor_recording
for case_name in topic_a buffered_a buffered_b topic_b; do
  mode=${case_name%_*}
  target=artifacts/jetson_a3/timing_cost_v1_${case_name}
  python3 scripts/run_jetson_replay_trial.py "$recording" --output "$target" \
    --rate 1 --executor-mode legacy --timing-mode "$mode" \
    --trace-wheels-publication --trace-imu-publication --dds-transport shm_16m \
    --dds-audit-library build/libarena_dds_probe_v1.so
done
echo TIMING_COST_FOUR_TRIALS_COMPLETE
