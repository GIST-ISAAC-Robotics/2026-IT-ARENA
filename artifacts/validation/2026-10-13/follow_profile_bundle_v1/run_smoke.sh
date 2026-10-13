#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
repo="$PWD"
base="$repo/artifacts/validation/2026-10-13/follow_profile_bundle_v1"
test ! -e "$base/smoke.log"
python3 scripts/verify_follow_profile_bundle.py build/jetson_g1_profile_20261013_v1.tar.gz "$base/manifest.json" --output "$base/preextract_verification.json" > "$base/preextract.log" 2>&1
bundle_dir=$(mktemp -d "$repo/build/follow-profile-smoke.XXXXXX")
case "$bundle_dir" in "$repo"/build/follow-profile-smoke.*) ;; *) exit 2 ;; esac
tar --extract --gzip --file "$repo/build/jetson_g1_profile_20261013_v1.tar.gz" --directory "$bundle_dir" --no-same-owner --no-same-permissions
cd "$bundle_dir"
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts"
python3 -B - <<'PY' > "$base/smoke.log" 2>&1
from pathlib import Path
import arena_autonomy.local_follow as follow
import arena_vehicle_interface.drive_feedback as feedback
import profile_follow_trace as profile
root = Path.cwd().resolve()
for module in (follow, feedback, profile):
    source = Path(module.__file__).resolve()
    assert source.is_relative_to(root), source
    print('isolated_import', source)
PY
python3 -B scripts/audit_hardware_capture.py config/tests/hardware_capture_nominal_v1.jsonl --output "$base/isolated_capture_report.json" >> "$base/smoke.log" 2>&1
python3 -B scripts/profile_follow_trace.py artifacts/validation/2026-10-13/gapped_boundary_v3/side_left_s0/follow_decisions.jsonl --input-mode stream --warmups 0 --repeats 1 --output artifacts/validation/local_package_smoke > "$base/smoke_profile.log" 2>&1
cp artifacts/validation/local_package_smoke/report.json "$base/isolated_profile_report.json"
printf '%s\n' "$bundle_dir" >> "$base/smoke.log"
tail -5 "$base/smoke.log"
