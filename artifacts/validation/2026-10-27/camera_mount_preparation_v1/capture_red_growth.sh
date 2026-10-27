#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/camera_mount_preparation_v1
/usr/bin/python3 -c 'import sys,rclpy; print(sys.executable, rclpy.__file__)' > "$base/red_growth_environment.log"
for name in render_h25_red0_env2 render_h25_red1 render_h25_red2 render_h25_red3; do
    /usr/bin/python3 -B scripts/capture_grid_start.py "$base/$name" > "$base/${name}_capture.log" 2>&1
done
