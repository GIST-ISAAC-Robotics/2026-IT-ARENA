#!/usr/bin/env bash
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
ARTIFACT="$ROOT/artifacts/validation/2026-10-17/g1_recent_role_shadow_v1"
cd "$ROOT"
mkdir -p "$ARTIFACT"
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ARTIFACT/test_source_before.sha256"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
python3 -B -m pytest -q tests/test_corridor_role_probe.py > "$ARTIFACT/test.log" 2>&1
status=$?
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ARTIFACT/test_source_after.sha256"
diff -u "$ARTIFACT/test_source_before.sha256" "$ARTIFACT/test_source_after.sha256" > "$ARTIFACT/test_source.diff" || true
echo "EXIT_STATUS=$status" > "$ARTIFACT/test.status"
exit "$status"
