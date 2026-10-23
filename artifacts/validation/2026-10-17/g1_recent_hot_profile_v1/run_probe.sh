#!/usr/bin/env bash
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
ART="$ROOT/artifacts/validation/2026-10-17/g1_recent_hot_profile_v1"
cd "$ROOT"
printf '%s\n' 'python3 -B artifacts/validation/2026-10-17/g1_recent_hot_profile_v1/probe.py' > "$ART/command.txt"
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ART/source_hashes_before.sha256"
sha256sum artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl artifacts/validation/2026-10-17/g1_recent_full_profile_stream_v1/report.json > "$ART/inputs_before.sha256"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
set +e
python3 -B artifacts/validation/2026-10-17/g1_recent_hot_profile_v1/probe.py > "$ART/stdout.log" 2>&1
status=$?
set -e
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ART/source_hashes_after.sha256"
sha256sum artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl artifacts/validation/2026-10-17/g1_recent_full_profile_stream_v1/report.json > "$ART/inputs_after.sha256"
diff -u "$ART/source_hashes_before.sha256" "$ART/source_hashes_after.sha256" > "$ART/source_hash_diff.txt" || true
diff -u "$ART/inputs_before.sha256" "$ART/inputs_after.sha256" > "$ART/inputs_hash_diff.txt" || true
printf 'EXIT_STATUS=%s\n' "$status" > "$ART/runner_status.txt"
cat "$ART/stdout.log"
exit "$status"
