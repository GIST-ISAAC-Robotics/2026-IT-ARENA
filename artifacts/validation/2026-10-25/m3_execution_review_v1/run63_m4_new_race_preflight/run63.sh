#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
D=artifacts/validation/2026-10-25/m3_execution_review_v1/run63_m4_new_race_preflight
[ ! -f "$D/test_exit_code.txt" ] || exit 91
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="scripts:src/arena_autonomy:src/arena_vehicle_interface:tests:${PYTHONPATH:-}"
python3 -B -m pytest -q tests/test_m4_operation_trial.py tests/test_m4_operation_cli.py tests/test_race_session.py tests/test_race_session_node.py tests/test_source_provenance_r0_plus_b3.py tests/test_g1_dense_stop.py tests/test_corner_follow.py tests/test_physics_step_option.py > "$D/test_stdout.txt" 2> "$D/test_stderr.txt"
code=$?; printf '%s\n' "$code" > "$D/test_exit_code.txt"
[ "$code" = 0 ] || exit "$code"
python3 - <<'PY' > "$D/review_stdout.txt" 2> "$D/review_stderr.txt"
import json,hashlib
from pathlib import Path
from m4_operation_eval import evaluate_trial
root=Path('artifacts/validation/2026-10-25/m3_execution_review_v1')
run=root/'run61_b3_same_race/run'
names=['report.json','m4_operation_trace.json','ego_poses.jsonl','actuation_trace.json']
h=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before={n:h(run/n) for n in names}
load=lambda n:json.loads((run/n).read_text())
r=load('report.json')
result=evaluate_trial(load('m4_operation_trace.json'),r,[json.loads(l) for l in (run/'ego_poses.jsonl').read_text().splitlines()],load('actuation_trace.json'))
result['input_sha256']=before
result['inputs_unchanged']=all(h(run/n)==v for n,v in before.items())
result['evaluator_sha256']=h(Path('scripts/m4_operation_eval.py'))
assert result['inputs_unchanged']
(root/'run63_m4_new_race_preflight/run61_reassessment.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k in ('passed','gates','clear_transition','restart_delay_s','progress_after_rearm_m','inputs_unchanged')},ensure_ascii=False))
PY
code=$?; printf '%s\n' "$code" > "$D/review_exit_code.txt"
echo "RUN63_DONE test=$(cat "$D/test_exit_code.txt") review=$code"
exit "$code"