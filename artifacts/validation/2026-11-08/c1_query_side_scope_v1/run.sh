#!/usr/bin/env bash
# 사용: bash run.sh A   또는   bash run.sh B   (LF 전용, 한 번에 하나. 각 단계 실제 종료 코드를 기록한다)
# A=A_no_injection, B=B_c1_injection. 같은 registered.json·source_snapshot.zip을 쓰며 출력 폴더가 서로 다르다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
P=artifacts/validation/2026-11-08/c1_query_side_scope_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
K=${1:?A|B}
case "$K" in A) D="$P/A_no_injection" ;; B) D="$P/B_c1_injection" ;; *) echo 'A|B'; exit 2 ;; esac
if [[ ! -f "$P/registered.json" || ! -d "$D" || -e "$D/run1" || -e "$D/run1.exit" ]]; then echo 'not registered or already run'; exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/preflight_processes_run1.txt"; then
  echo "PREFLIGHT_BUSY=1" > "$D/run1.exit"; exit 3
fi
start=$(date +%s)
bash "$ENV" /usr/bin/python3 -B "$P/run.py" "$K" > "$D/run1_console.log" 2>&1
harness=$?
surface=-1
if [[ -f "$D/run1/report.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B scripts/surface_contact_eval.py "$D/run1" --output "$D/surface_run1.json" > "$D/surface_run1.log" 2>&1
  surface=$?
fi
/usr/bin/python3 -B - "$P" "$D" <<'PY'
import hashlib, json, sys
from pathlib import Path
p, d = Path(sys.argv[1]), Path(sys.argv[2]); r = json.loads((p/'registered.json').read_text(encoding='utf-8'))
changed = [k for k, h in r['sources'].items() if hashlib.sha256(Path(k).read_bytes()).hexdigest() != h]
snapshot = hashlib.sha256((p/'source_snapshot.zip').read_bytes()).hexdigest() == r['snapshot_sha256']
(d/'postflight_integrity_run1.json').write_text(json.dumps({'inputs': len(r['sources']), 'changed': changed, 'snapshot_unchanged': snapshot}, indent=2)+'\n')
raise SystemExit(0 if not changed and snapshot else 1)
PY
integrity=$?
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$D/postflight_processes_run1.txt"; leftover=$?
assessment=-1
if [[ -f "$D/run1/report.json" ]]; then
  bash "$ENV" /usr/bin/python3 -B "$P/assess.py" "$K" > "$D/assessment_run1.log" 2>&1
  assessment=$?
fi
echo "HARNESS_EXIT=$harness SURFACE_EXIT=$surface INTEGRITY_EXIT=$integrity LEFTOVER_PGREP=$leftover ASSESSMENT_EXIT=$assessment WALL_S=$(( $(date +%s)-start ))" > "$D/run1.exit"
exit "$harness"
