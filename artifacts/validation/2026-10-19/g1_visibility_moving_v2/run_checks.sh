#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
out=artifacts/validation/2026-10-19/g1_visibility_moving_v2
if [ -e "$out/pytest.log" ]; then exit 2; fi
find src scripts tests config -type f \( -name '*.py' -o -name '*.json' -o -name '*.yaml' \) -print0 | sort -z | xargs -0 sha256sum > "$out/source_before.sha256"
set +e
python3 -B -m pytest -q -p no:cacheprovider tests/test_corner_visibility_grid.py tests/test_front_unknown_diagnostic.py tests/test_corner_path_road.py > "$out/pytest.log" 2>&1
code=$?
printf '\nPYTEST_EXIT_CODE=%s\n' "$code" >> "$out/pytest.log"
if [ "$code" -ne 0 ]; then tail -n 45 "$out/pytest.log"; exit "$code"; fi
python3 -B scripts/probe_corner_visibility_grid.py artifacts/validation/2026-10-17/g1_recent_solo_v1/run --output "$out/run" --history-mode steady-arc > "$out/probe.log" 2>&1
code=$?
printf '\nPROBE_EXIT_CODE=%s\n' "$code" >> "$out/probe.log"
find src scripts tests config -type f \( -name '*.py' -o -name '*.json' -o -name '*.yaml' \) -print0 | sort -z | xargs -0 sha256sum > "$out/source_after.sha256"
diff -u "$out/source_before.sha256" "$out/source_after.sha256" > "$out/source_hash_diff.txt"
hash_code=$?
tail -n 4 "$out/pytest.log"
tail -n 8 "$out/probe.log"
if [ "$code" -ne 0 ]; then exit "$code"; fi
exit "$hash_code"
