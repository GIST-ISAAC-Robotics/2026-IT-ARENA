"""launch 가드를 직접 불러 slow_lead run1에서 난 거부가 새 공식 track에서 풀리고 나머지 조건은 그대로인지 확인한다."""
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
spec = importlib.util.spec_from_file_location('guard_check_launch', ROOT/'src/arena_bringup/launch/simulation.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)
base = dict(guarded=True, actor='evaluation_actor.sdf', evidence='run_dir')
rows = {}
for track in ('official', 'official_v2026_11_06', 'experimental', 'original'):
    for label, override in (('ok', {}), ('unguarded', {'guarded': False}), ('no_actor', {'actor': ''}),
                            ('no_evidence', {'evidence': ''})):
        try:
            rows[f'{track}/{label}'] = launch._autonomy_executable('local_pursuit', 'paced', **{**base, **override}, track=track)
        except ValueError as error:
            rows[f'{track}/{label}'] = f'ValueError: {error}'
expected_ok = {'official/ok', 'official_v2026_11_06/ok'}
passed = all((v == 'g1_g2_handoff_trial') == (k in expected_ok) for k, v in rows.items())
print(json.dumps(dict(passed=passed, results=rows), ensure_ascii=False, indent=1))
raise SystemExit(0 if passed else 1)
