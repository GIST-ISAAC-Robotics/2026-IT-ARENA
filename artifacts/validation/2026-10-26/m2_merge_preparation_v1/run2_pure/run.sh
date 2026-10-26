set -u
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out="$repo/artifacts/validation/2026-10-26/m2_merge_preparation_v1/run2_pure"
cd "$repo" || exit 90
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface:$repo/scripts:$repo/tests:${PYTHONPATH:-}"
printf 'repo=%s\nPYTHONPATH=%s\n' "$repo" "$PYTHONPATH"
python3 -c 'import numpy, pytest, arena_autonomy; print("numpy", numpy.__version__); print("pytest", pytest.__version__); print("arena_autonomy", list(arena_autonomy.__path__))' || exit 91
python3 -m pytest -q scripts/traffic_merge.py tests/test_traffic_merge.py
exitcode=$?
printf '%s\n' "$exitcode" > "$out/exitcode.txt"
exit "$exitcode"