"""d1 입력 불일치 보존 후 한 번만 허용한 동일 Dantzig 재실행. 자동 retry 없음."""
import json
from pathlib import Path
import sys
import zipfile
from analyze_evaluator_gc import sha
from run_probe_batch import run_plan

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
prior = json.loads((BASE/'registered_manifest.json').read_text())
for path, expected in prior['sources'].items():
    if sha(ROOT/path) != expected:
        raise SystemExit('registered source changed: '+path)
if sha(ROOT/'install/arena_gazebo/lib/libarena-single-motor-system.so') != prior['plugin_sha256']:
    raise SystemExit('binary changed')
plan = {'base': BASE.relative_to(ROOT).as_posix(), 'cases': [
    {'name': 'd2_dantzig', 'expect': 'pass', 'args': ['--solver', 'dantzig']}]}
with (BASE/'repeat_registration.json').open('x') as f:
    json.dump({'reason': 'd1/p1 applied envelopes differ from 3.855/3.856 s; one bounded retry, no tuning',
        'plan': plan, 'parent_sha256': sha(BASE/'registered_manifest.json'),
        'source_sha256': {p.relative_to(ROOT).as_posix(): sha(p) for p in (Path(__file__), BASE/'repeat_dantzig.sh')},
        'prior_analysis_sha256': sha(BASE/'analysis_v2.json')}, f, indent=2)
raise SystemExit(run_plan(plan, BASE/'status_d2_dantzig.json',
    command=[sys.executable, '-B', 'scripts/probe_solver_compare.py']))
