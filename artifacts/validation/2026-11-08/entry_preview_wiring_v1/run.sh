#!/usr/bin/env bash
# r36 1회. GPT가 후보·연결 적용, 빌드, r36 출처 계약을 끝내고 r35 실행이 끝난 뒤에만 사용한다.
# --check는 등록·실행 없이 전제만 확인한다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/entry_preview_wiring_v1
ENV=artifacts/validation/2026-11-01/turn_preview_v1/environment.sh
if [ "${1:-}" = "--check" ]; then
    python3 -B -I "$A/check_wiring.py" > /dev/null || { echo 'wiring check failed'; exit 6; }
    python3 -B - "$A" <<'PY' || exit 5
import hashlib,json,sys
from pathlib import Path
a=Path(sys.argv[1]);m=json.loads((a/'wiring_manifest.json').read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for label,table in (('before',{**m['candidate_overlay']['before'],**m['before']}),('after',{**m['candidate_overlay']['after'],**m['after']})):
    state={p:Path(p).is_file() and sha(p)==h for p,h in table.items()}
    print(label,'all' if all(state.values()) else 'partial' if any(state.values()) else 'none',sum(state.values()),'/',len(state))
PY
    pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' && echo 'simulation running' || echo 'no simulation'
    [ -e "$A/run1" ] && echo 'run1 exists' || echo 'run1 absent'
    exit 0
fi
if [ -e "$A/run1" ]; then exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/preflight.txt"; then exit 3; fi
bash "$ENV" python3 -B "$A/register.py" > "$A/register.log" 2>&1
status=$?; printf '%s\n' "$status" > "$A/register.exit"; [ "$status" -eq 0 ] || exit "$status"
bash "$ENV" python3 -B "$A/run.py" > "$A/run.log" 2>&1
status=$?; printf '%s\n' "$status" > "$A/run.exit"
if [ -f "$A/run1/report.json" ]; then
    bash "$ENV" python3 -B "$A/assess.py" > "$A/assess.log" 2>&1
    assessment=$?; printf '%s\n' "$assessment" > "$A/assess.exit"
fi
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/postflight.txt"
[ "$status" -eq 0 ] || exit "$status"
exit "${assessment:-1}"
