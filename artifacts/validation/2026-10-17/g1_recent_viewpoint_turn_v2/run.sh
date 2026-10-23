#!/usr/bin/env bash
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
ART="$ROOT/artifacts/validation/2026-10-17/g1_recent_viewpoint_turn_v2"
cd "$ROOT"
printf '%s\n' 'python3 -B artifacts/validation/2026-10-17/g1_recent_viewpoint_turn_v2/probe.py' > "$ART/command.txt"
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ART/runtime_hashes_before.sha256"
find artifacts/validation/2026-10-17/g1_recent_solo_v1/run -maxdepth 1 -type f -print0 | sort -z | xargs -0 sha256sum > "$ART/input_hashes_before.sha256"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
set +e
python3 -B artifacts/validation/2026-10-17/g1_recent_viewpoint_turn_v2/probe.py > "$ART/stdout.log" 2>&1
status=$?
set -e
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ART/runtime_hashes_after.sha256"
find artifacts/validation/2026-10-17/g1_recent_solo_v1/run -maxdepth 1 -type f -print0 | sort -z | xargs -0 sha256sum > "$ART/input_hashes_after.sha256"
diff -u "$ART/runtime_hashes_before.sha256" "$ART/runtime_hashes_after.sha256" > "$ART/runtime_hash_diff.txt" || true
diff -u "$ART/input_hashes_before.sha256" "$ART/input_hashes_after.sha256" > "$ART/input_hash_diff.txt" || true
printf 'EXIT_STATUS=%s\n' "$status" > "$ART/runner_status.txt"
cat "$ART/stdout.log"
exit "$status"
