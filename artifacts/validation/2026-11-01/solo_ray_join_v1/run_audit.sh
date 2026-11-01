#!/usr/bin/env bash
# 호출: bash run_audit.sh <variant>
cd "$(dirname "${BASH_SOURCE[0]}")"
ROOT=$(cd ../../../.. && pwd)
bash ../low_speed_arc_reacquire_v1/env.sh python3 -B artifacts/validation/2026-11-01/solo_ray_join_v1/join_audit.py "$1" "$ROOT/artifacts/validation/2026-11-01/solo_corridor_arc_v1/run1" "$ROOT/artifacts/validation/2026-11-01/solo_ray_join_v1/audit_$1.json" > "audit_$1.log" 2>&1
echo "EXIT=$?" > "audit_$1.exit"
