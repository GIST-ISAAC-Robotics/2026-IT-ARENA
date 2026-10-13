#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/scripts"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/hardware_capture_audit_v2
test ! -e "$base/before.log"
python3 -B "$base/probe_before.py" --auditor build/follow-profile-smoke.JCenu7/scripts/audit_hardware_capture.py --example config/tests/hardware_capture_nominal_v1.jsonl --output "$base/before.json" > "$base/before.log" 2>&1
tail -1 "$base/before.log"
