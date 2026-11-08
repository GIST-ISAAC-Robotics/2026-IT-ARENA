#!/usr/bin/env bash
# 사용: wsl.exe -e bash <this> <slow_lead|straight_moving_lead>   (LF 전용. 각 단계 실제 종료 코드를 기록한다)
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-07/traffic_surface_alignment_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
N=${1:?case}; D="$A/$N"
if [[ ! -f "$D/registered.json" || -e "$D/run2" || -e "$D/run2.exit" ]]; then echo 'not registered or already run'; exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/preflight_processes_run2.txt"; then
  echo "PREFLIGHT_BUSY=1" > "$D/run2.exit"; exit 3
fi
start=$(date +%s)
bash "$ENV" /usr/bin/python3 -B "$A/run.py" "$N" > "$D/run2_console.log" 2>&1
harness=$?
surface=-1
if [[ -f "$D/run2/report.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$D/run2" --output "$D/surface_run2.json" > "$D/surface_run2.log" 2>&1
  surface=$?
fi
/usr/bin/python3 -B - "$D" <<'PY'
import hashlib, json, sys
from pathlib import Path
d = Path(sys.argv[1]); r = json.loads((d/'registered.json').read_text(encoding='utf-8'))
changed = [p for p, h in r['sources'].items() if hashlib.sha256(Path(p).read_bytes()).hexdigest() != h]
(d/'postflight_integrity_run2.json').write_text(json.dumps({'inputs': len(r['sources']), 'changed': changed}, indent=2)+'\n')
PY
integrity=$?
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/postflight_processes_run2.txt"; leftover=$?
assessment=-1
if [[ -f "$D/run2/report.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B "$A/assess.py" "$N" > "$D/assessment_run2.log" 2>&1
  assessment=$?
fi
echo "HARNESS_EXIT=$harness SURFACE_EXIT=$surface INTEGRITY_EXIT=$integrity LEFTOVER_PGREP=$leftover ASSESSMENT_EXIT=$assessment WALL_S=$(( $(date +%s)-start ))" > "$D/run2.exit"
exit "$harness"
