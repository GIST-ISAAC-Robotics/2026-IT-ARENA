set -u
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out="$repo/artifacts/validation/2026-10-26/m2_merge_preparation_v1/run3_route_audit"
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
if [ -f install/setup.bash ]; then source install/setup.bash || exit 92; fi
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface:$repo/scripts:$repo/tests:${PYTHONPATH:-}"
printf 'repo=%s\nPYTHONPATH=%s\n' "$repo" "$PYTHONPATH"
python3 -c 'import numpy, pytest, arena_autonomy; print("numpy", numpy.__version__); print("pytest", pytest.__version__); print("arena_autonomy", list(arena_autonomy.__path__))' || exit 93
python3 -I -B "$repo/artifacts/validation/2026-10-26/m2_merge_preparation_v1/audit_route.py" "$out/data"
rc=$?
printf '%s\n' "$rc" > "$out/exitcode.txt"
exit "$rc"