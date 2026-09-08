#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
for mode in baseline bullet bullet_no_scene; do
  args=()
  if [[ "$mode" != baseline ]]; then args+=(--collision-detector bullet); fi
  if [[ "$mode" == bullet_no_scene ]]; then args+=(--no-scene-broadcaster); fi
  python3 scripts/smoke_simulation.py --d435i-profile low_load_30 --camera-seconds 3 --render-backend software "${args[@]}" --output-dir "artifacts/validation/2026-09-08/performance/$mode" > "artifacts/validation/2026-09-08/performance/${mode}_console.log" 2>&1
  printf '%s %s\n' "$mode" "$?"
done
