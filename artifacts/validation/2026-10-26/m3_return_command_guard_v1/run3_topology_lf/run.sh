#!/usr/bin/env bash
set -uo pipefail
repo="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
cd "$repo" || exit 90
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface:$repo/scripts:$repo/tests"
tests=(tests/test_scan_topology.py tests/test_local_scene.py tests/test_observation_support.py tests/test_observation_depth_window.py tests/test_observation_depth_equivalence.py tests/test_observation_depth_cone.py tests/test_observation_depth_chunk.py tests/test_m3_baseline_release.py tests/test_m3_release_gate_review.py tests/test_m3_retained_threats.py tests/test_m3_memory_geometry.py tests/test_m3_return_guard.py)
for f in "${tests[@]}"; do
  test -f "$f" || { printf "MISSING_PATH %q\n" "$f" >&2; exit 92; }
done
python3 "$repo/artifacts/validation/2026-10-26/m3_return_command_guard_v1/run3_topology_lf/preflight.py"
python3 -c 'import numpy,pytest,arena_autonomy; print("numpy",numpy.__version__); print("pytest",pytest.__version__); print("arena_autonomy.__path__",list(arena_autonomy.__path__))'
python3 -m pytest -q "${tests[@]}"
