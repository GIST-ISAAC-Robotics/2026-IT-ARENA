#!/usr/bin/env bash
set -euo pipefail

cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"

output="artifacts/validation/2026-10-04/pipeline_fault_validation/gazebo_safety_exit_v1"
copy_renderer_log() {
  if [[ -d "$output" && -f /home/jinhyeong/.gz/rendering/ogre2.log ]]; then
    cp /home/jinhyeong/.gz/rendering/ogre2.log "$output/ogre2.log" || true
  fi
}
trap copy_renderer_log EXIT

set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u

export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"

python3 -m pytest -q tests/test_lidar_shutdown.py 2>&1 | tee artifacts/validation/2026-10-04/pipeline_fault_validation/shutdown_unit_v1.log
colcon build --base-paths src --packages-select arena_autonomy arena_bringup --symlink-install 2>&1 | tee artifacts/validation/2026-10-04/pipeline_fault_validation/colcon_build_v1.log
python3 scripts/validate_basic_autonomy.py \
  --autonomy-mode local_pursuit --lidar-acquisition sequential --lidar-compensation both \
  --lidar-rate-hz 10 --control-rate-hz 50 --speed-profile local_fast --disable-speed-bump \
  --actuation-mode virtual_mcu --actuation-stop safety_exit --shutdown-mode service \
  --render-backend wsl_nvidia --sensor-wall-timeout-s 30 --stop-wall-timeout-s 120 \
  --grid-slot 3 --target-progress-m 12 --max-sim-seconds 60 --wall-timeout-s 900 \
  --output "$output" 2>&1 | tee artifacts/validation/2026-10-04/pipeline_fault_validation/gazebo_safety_exit_v1_console.log
