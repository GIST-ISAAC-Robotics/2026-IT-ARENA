#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/current_corner_fallback_r47_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if [ "${1:-}" = "--check" ]; then
  bash "$ENV" python3 -B "$A/register.py" --check || exit $?
  bash "$ENV" python3 -B "$A/check_runtime.py" --dry
  exit $?
fi
[ ! -e "$A/registered.json" ] && [ ! -e "$A/run1" ] || exit 4
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/preflight.txt"; then exit 3; fi
start=$(date +%s)
bash "$ENV" python3 -B "$A/register.py" > "$A/register.log" 2>&1
registered=$?; printf '%s\n' "$registered" > "$A/register.exit"; [ "$registered" -eq 0 ] || exit "$registered"
bash "$ENV" python3 -B "$A/check_runtime.py" CASE > "$A/runtime_check.log" 2>&1 || exit 6
bash "$ENV" python3 -B "$A/run.py" > "$A/run.log" 2>&1
harness=$?
surface=-1
if [ -f "$A/run1/report.json" ]; then
  bash "$ENV" python3 -B scripts/surface_contact_eval.py "$A/run1" --output "$A/surface_run1.json" > "$A/surface_run1.log" 2>&1
  surface=$?
fi
python3 -B - "$A" <<'PY'
from pathlib import Path
import hashlib,json,sys
a=Path(sys.argv[1]);r=json.loads((a/'registered.json').read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
changed=[p for p,h in r['sources'].items() if not Path(p).is_file() or sha(p)!=h]
out=dict(inputs=len(r['sources']),changed=changed,snapshot_unchanged=sha(a/'source_snapshot.zip')==r['snapshot_sha256'])
(a/'postflight_integrity_run1.json').write_text(json.dumps(out,indent=2)+'\n')
raise SystemExit(0 if not changed and out['snapshot_unchanged'] else 1)
PY
integrity=$?
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/postflight_processes_run1.txt"
remaining=$?
assessment=-1
if [ -f "$A/run1/report.json" ]; then
  bash "$ENV" python3 -B "$A/assess.py" > "$A/assessment_run1.log" 2>&1
  assessment=$?
fi
printf 'HARNESS_EXIT=%s SURFACE_EXIT=%s INTEGRITY_EXIT=%s LEFTOVER_PGREP=%s ASSESSMENT_EXIT=%s WALL_S=%s\n' "$harness" "$surface" "$integrity" "$remaining" "$assessment" "$(( $(date +%s)-start ))" > "$A/run1.exit"
[ "$integrity" -eq 0 ] && [ "$remaining" -eq 1 ] || exit 7
[ "$harness" -eq 0 ] || exit "$harness"
exit "$assessment"
