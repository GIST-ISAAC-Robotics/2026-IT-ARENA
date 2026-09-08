#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
cmake --build build/performance_native -j2
for case_name in static_full physics_only no_physics empty_physics; do
  GZ_PARTITION="arena_profile_${case_name}" timeout 90 build/performance_native/arena_profile "artifacts/validation/2026-09-08/performance/${case_name}.sdf" > "artifacts/validation/2026-09-08/performance/${case_name}.log" 2>&1
done
