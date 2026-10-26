set +u
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
o="$repo/artifacts/validation/2026-10-26/m2_motion_stress_v1/run1_contract_final"
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/setup.bash || exit 92
export PYTHONPATH="$repo/tests:$repo/scripts:$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface:${PYTHONPATH:-}"
python3 -m pytest -q tests/test_source_provenance_r0_plus_b3.py > "$o/stdout.txt" 2> "$o/stderr.txt"
rc=$?
printf '%s\n' "$rc" > "$o/exit_code.txt"
exit "$rc"