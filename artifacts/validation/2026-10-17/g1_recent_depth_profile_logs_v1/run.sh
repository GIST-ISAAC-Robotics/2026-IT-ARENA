#!/usr/bin/env bash
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
LOG="$ROOT/artifacts/validation/2026-10-17/g1_recent_depth_profile_logs_v1"
OUT="$ROOT/artifacts/validation/2026-10-17/g1_recent_depth_full_profile_v1"
cd "$ROOT"
mkdir -p "$LOG"
test ! -e "$OUT"
printf '%s\n' 'python3 -B scripts/profile_follow_trace.py artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl --output artifacts/validation/2026-10-17/g1_recent_depth_full_profile_v1 --repeats 1 --warmups 0 --input-mode stream' > "$LOG/command.txt"
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$LOG/source_hashes_before.sha256"
sha256sum artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl > "$LOG/input_before.sha256"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
set +e
python3 -B scripts/profile_follow_trace.py artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl --output artifacts/validation/2026-10-17/g1_recent_depth_full_profile_v1 --repeats 1 --warmups 0 --input-mode stream > "$LOG/stdout.log" 2>&1
status=$?
set -e
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$LOG/source_hashes_after.sha256"
sha256sum artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl > "$LOG/input_after.sha256"
diff -u "$LOG/source_hashes_before.sha256" "$LOG/source_hashes_after.sha256" > "$LOG/source_hash_diff.txt" || true
diff -u "$LOG/input_before.sha256" "$LOG/input_after.sha256" > "$LOG/input_hash_diff.txt" || true
printf 'EXIT_STATUS=%s\n' "$status" > "$LOG/runner_status.txt"
cat "$LOG/stdout.log"
exit "$status"
