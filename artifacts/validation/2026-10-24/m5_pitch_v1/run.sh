#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PYTHONPATH"
out=artifacts/validation/2026-10-24/m5_pitch_v1
sha256sum scripts/audit_grid_camera_pitch.py scripts/audit_start_signal_geometry.py tests/test_grid_camera_pitch.py src/arena_description/config/b0183_c1.yaml artifacts/validation/2026-10-13/race_start_geometry_v1/result/scene.json artifacts/validation/2026-10-13/race_start_geometry_v1/result/world.sdf > "$out/before.sha256"
zip -q "$out/source.zip" scripts/audit_grid_camera_pitch.py scripts/audit_start_signal_geometry.py tests/test_grid_camera_pitch.py
set +e
python3 -m pytest -q tests/test_grid_camera_pitch.py tests/test_start_signal_geometry.py tests/test_grid_start.py --junitxml="$out/junit.xml" > "$out/tests.log" 2>&1
code=$?
if [ "$code" -eq 0 ]; then
    python3 scripts/audit_grid_camera_pitch.py --output "$out/result" > "$out/analysis.log" 2>&1
    code=$?
fi
set -e
printf '%s\n' "$code" > "$out/exit.txt"
sha256sum -c "$out/before.sha256" > "$out/source_check.txt"
exit "$code"
