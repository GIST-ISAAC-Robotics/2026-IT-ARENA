#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/../../../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
for backend in software wsl_nvidia; do
  python3 scripts/validate_lidar_control_lab.py --case straight_5kmh --rate 10 --acquisition sequential --source-rate 500 --render-backend "$backend" --output "artifacts/validation/2026-09-08/gpu_final/sequential_$backend" || code=$?
  cp "$HOME/.gz/rendering/ogre2.log" "artifacts/validation/2026-09-08/gpu_final/sequential_$backend/ogre2.log"
done
