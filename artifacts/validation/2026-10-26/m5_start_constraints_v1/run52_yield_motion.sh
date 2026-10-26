#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
B=artifacts/validation/2026-10-26/m5_start_constraints_v1
O="$B/run52_yield_motion"
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
cp scripts/probe_grid_deferred_center.py "$O/probe_source.py"
cp tests/test_deferred_center_reference.py "$O/test_source.py"
python3 -B -m pytest -q tests/test_deferred_center_reference.py > "$O/tests_stdout.txt" 2> "$O/tests_stderr.txt"
result=$?
printf '%s\n' "$result" > "$O/tests_exit_code.txt"
if [ "$result" -eq 0 ]; then
python3 -B - "$B" "$O" <<'PY' > "$O/comparison_stdout.txt" 2> "$O/comparison_stderr.txt"
import json,sys,hashlib
from pathlib import Path
from probe_grid_deferred_center import rejected_yield_diagnostic_only
base,out=map(Path,sys.argv[1:]);p=base/'run51_yield_motion/gazebo_saved_replay.json'
raw=p.read_bytes();r=json.loads(raw)
diagnostic=[m['seq'] for m in r['mismatches'] if rejected_yield_diagnostic_only(m['expected'],m['actual'])]
other=[m['seq'] for m in r['mismatches'] if m['seq'] not in diagnostic]
summary=dict(calls=r['calls'],full_output_matches=r['matched'],full_output_equal=r['passed'],
    added_rejected_yield_only=diagnostic,other_differences=other,input_unchanged=p.read_bytes()==raw,
    input_sha256=hashlib.sha256(raw).hexdigest(),scope='stored comparison classification; no new replay or physical run')
(out/'saved_gazebo_comparison.json').write_text(json.dumps(summary,indent=2))
assert r['input_unchanged'] and summary['input_unchanged'] and not other and r['calls']==810
PY
result=$?
fi
if [ "$result" -eq 0 ]; then
    python3 -B scripts/probe_grid_deferred_center.py --allow-rejected-yield-diagnostic --output "$O/data" > "$O/stdout.txt" 2> "$O/stderr.txt"
    result=$?
fi
printf '%s\n' "$result" > "$O/exit_code.txt"
date -Is > "$O/end.txt"
exit "$result"
