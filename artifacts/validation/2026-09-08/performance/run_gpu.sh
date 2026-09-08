#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/smoke_simulation.py --d435i-profile low_load_30 --camera-seconds 3 --render-backend wsl_nvidia --collision-detector bullet --no-scene-broadcaster --output-dir artifacts/validation/2026-09-08/performance/gpu_bullet_no_scene > artifacts/validation/2026-09-08/performance/gpu_bullet_no_scene_console.log 2>&1
result=$?
cp ~/.gz/rendering/ogre2.log artifacts/validation/2026-09-08/performance/gpu_bullet_no_scene_ogre2.log
exit "$result"
