#!/usr/bin/env bash
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
ART="$ROOT/artifacts/validation/2026-10-17/g1_recent_depth_replay_v1"
cd "$ROOT"
mkdir -p "$ART"
printf '%s\n' \
  'python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl --output artifacts/validation/2026-10-17/g1_recent_depth_replay_v1/oct17_on.json' \
  'python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl --output artifacts/validation/2026-10-17/g1_recent_depth_replay_v1/oct16_off.json' > "$ART/commands.txt"
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ART/source_hashes_before.sha256"
sha256sum artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl > "$ART/inputs_before.sha256"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
set +e
python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl --output "$ART/oct17_on.json" > "$ART/oct17.log" 2>&1
on_status=$?
if [ "$on_status" -eq 0 ]; then
  python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl --output "$ART/oct16_off.json" > "$ART/oct16.log" 2>&1
  off_status=$?
else
  off_status=125
  printf '%s\n' 'SKIPPED_AFTER_ON_FAILURE' > "$ART/oct16.log"
fi
set -e
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ART/source_hashes_after.sha256"
sha256sum artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl > "$ART/inputs_after.sha256"
diff -u "$ART/source_hashes_before.sha256" "$ART/source_hashes_after.sha256" > "$ART/source_hash_diff.txt" || true
diff -u "$ART/inputs_before.sha256" "$ART/inputs_after.sha256" > "$ART/inputs_hash_diff.txt" || true
printf 'OCT17_EXIT=%s\nOCT16_EXIT=%s\n' "$on_status" "$off_status" > "$ART/runner_status.txt"
cat "$ART/oct17.log" "$ART/oct16.log"
[ "$on_status" -eq 0 ] && [ "$off_status" -eq 0 ]
