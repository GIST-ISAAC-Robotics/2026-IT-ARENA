#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
A=artifacts/validation/2026-11-07/b0390_course_baseline_v1
if [ -e "$A/run1" ] || [ -e "$A/run1.exit" ]; then echo 'existing run; no rerun'; exit 4; fi
if pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/preflight_processes.txt"; then exit 3; fi
python3 -B - "$A" <<'PY'
import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1]);r=json.loads((p/'registered.json').read_bytes())
changed=[f for f,h in r['input_sha256'].items() if hashlib.sha256(Path(f).read_bytes()).hexdigest()!=h]
assert not changed,changed
PY
[ $? -eq 0 ] || exit 5
start=$(date +%s)
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B \
  scripts/run_current_solo.py "$A/run1" --track official_v2026_11_06 \
  --local-sensor-config src/arena_description/config/b0390_c1_jr60p20_sim.yaml > "$A/run1_console.log" 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code WALL_S=$(( $(date +%s) - start ))" > "$A/run1.exit"
python3 -B - "$A" <<'PY'
import hashlib,json,sys
from pathlib import Path
p=Path(sys.argv[1]);r=json.loads((p/'registered.json').read_bytes())
changed=[f for f,h in r['input_sha256'].items() if hashlib.sha256(Path(f).read_bytes()).hexdigest()!=h]
(p/'postflight_integrity.json').write_text(json.dumps({'inputs':len(r['input_sha256']),'changed':changed},indent=2))
PY
pgrep -af '^(gz sim|/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch)' > "$A/postflight_processes.txt"
exit "$code"
