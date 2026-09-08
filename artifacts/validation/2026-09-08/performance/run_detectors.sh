#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
python3 artifacts/validation/2026-09-08/performance/detectors.py
for detector in ode bullet fcl dart; do
  GZ_PARTITION="arena_detector_${detector}" timeout 90 build/performance_native/arena_profile "artifacts/validation/2026-09-08/performance/detector_${detector}.sdf" > "artifacts/validation/2026-09-08/performance/detector_${detector}.log" 2>&1
  printf '%s %s\n' "$detector" "$?"
done
