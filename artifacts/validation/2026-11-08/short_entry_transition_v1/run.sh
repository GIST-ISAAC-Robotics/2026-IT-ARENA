#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/short_entry_transition_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if [ "${1:-}" = "--check" ]; then
  bash "$ENV" python3 -B "$A/register.py" --check || exit $?
  bash "$ENV" python3 -B "$A/check_runtime.py" --dry
  exit $?
fi
[ ! -e "$A/run1" ] && [ ! -e "$A/registered.json" ] || exit 4
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/preflight.txt"; then exit 3; fi
bash "$ENV" python3 -B "$A/register.py" > "$A/register.log" 2>&1
status=$?; printf '%s\n' "$status" > "$A/register.exit"; [ "$status" -eq 0 ] || exit "$status"
bash "$ENV" python3 -B "$A/check_runtime.py" OPS > "$A/runtime_check.log" 2>&1 || exit 6
bash "$ENV" python3 -B "$A/run.py" > "$A/run.log" 2>&1
status=$?; printf '%s\n' "$status" > "$A/run.exit"
if [ -f "$A/run1/report.json" ]; then
  bash "$ENV" python3 -B "$A/assess.py" > "$A/assess.log" 2>&1
  assessment=$?; printf '%s\n' "$assessment" > "$A/assess.exit"
fi
python3 -B - "$A" <<'PY'
from pathlib import Path
import hashlib,json,sys
a=Path(sys.argv[1]);r=json.loads((a/'registered.json').read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
changed=[p for p,h in r['sources'].items() if not Path(p).is_file() or sha(p)!=h]
out=dict(inputs=len(r['sources']),changed=changed,snapshot_unchanged=sha(a/'source_snapshot.zip')==r['snapshot_sha256'])
(a/'integrity.json').write_text(json.dumps(out,indent=2)+'\n')
raise SystemExit(0 if not changed and out['snapshot_unchanged'] else 1)
PY
integrity=$?; printf '%s\n' "$integrity" > "$A/integrity.exit"
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/postflight.txt"
remaining=$?
[ "$status" -eq 0 ] || exit "$status"
[ "$integrity" -eq 0 ] && [ "$remaining" -eq 1 ] || exit 7
exit "${assessment:-1}"
