#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
case_name="${1:-gpu_grace}"
python3 scripts/smoke_simulation.py --d435i-profile low_load_30 --camera-seconds 1.5 --shutdown-grace 70 --render-backend wsl_nvidia --collision-detector bullet --no-scene-broadcaster --output-dir "artifacts/validation/2026-09-08/performance/$case_name" > "artifacts/validation/2026-09-08/performance/${case_name}_console.log" 2>&1
result=$?
cp ~/.gz/rendering/ogre2.log "artifacts/validation/2026-09-08/performance/${case_name}_ogre2.log"
exit "$result"
