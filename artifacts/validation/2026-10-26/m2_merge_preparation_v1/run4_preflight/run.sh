set +u
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out="$repo/artifacts/validation/2026-10-26/m2_merge_preparation_v1/run4_preflight"
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/setup.bash || exit 92
export PYTHONPATH="$repo/scripts:$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface:$repo/tests:${PYTHONPATH:-}"
printf 'repo=%s\nPYTHONPATH=%s\n' "$repo" "$PYTHONPATH"
python3 -c 'import numpy, pytest, arena_autonomy; print("numpy", numpy.__version__); print("pytest", pytest.__version__); print("arena_autonomy", list(arena_autonomy.__path__))' || exit 93
python3 -m pytest -q tests/test_traffic_merge.py tests/test_traffic_road_route.py tests/test_traffic_actor_integration.py tests/test_traffic_actors.py tests/test_traffic_actor_runtime.py tests/test_source_provenance_r0_plus_b3.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py > "$out/pytest.stdout.txt" 2> "$out/pytest.stderr.txt"
rc=$?
printf '%s\n' "$rc" > "$out/pytest.exitcode.txt"
if [ "$rc" -ne 0 ]; then exit "$rc"; fi
python3 -I -B "$repo/artifacts/validation/2026-10-26/m2_merge_preparation_v1/audit_route.py" "$out/data" > "$out/audit.stdout.txt" 2> "$out/audit.stderr.txt"
rc=$?
printf '%s\n' "$rc" > "$out/audit.exitcode.txt"
exit "$rc"