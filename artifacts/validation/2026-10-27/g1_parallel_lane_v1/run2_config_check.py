"""run2 오른쪽 시험 설정이 run1 왼쪽 대조와 같은 조건인지 확인한다(읽기 전용).

허용 차이: 오른쪽 overlay의 옆차 `lateral_offset_m` 부호와 메타데이터, 오른쪽 그리드 설정의 `launch.grid_slot`(3→0)과
메타데이터. 그 밖의 차이·부모 해시 불일치·run1 설정 해시 불일치가 있으면 exit 1.
"""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
META = {'registered_at', 'scope', 'parent_overlay', 'parent_config'}


def load(rel):
    path = ROOT/rel
    return json.loads(path.read_text(encoding='utf-8')), hashlib.sha256(path.read_bytes()).hexdigest()


def flat(value, prefix=''):
    if isinstance(value, dict):
        out = {}
        for key, item in value.items():
            out.update(flat(item, f'{prefix}/{key}'))
        return out
    if isinstance(value, list):
        out = {}
        for index, item in enumerate(value):
            out.update(flat(item, f'{prefix}[{index}]'))
        return out
    return {prefix: value}


def differences(a, b):
    fa, fb = flat(a), flat(b)
    return {k: [fa.get(k), fb.get(k)] for k in sorted(set(fa) | set(fb)) if fa.get(k) != fb.get(k)}


def top(key):
    return key.split('/')[1].split('[')[0]


left, left_sha = load('config/tests/e3_onroad_encounter_v1.json')
right, right_sha = load('config/tests/e3_onroad_right_encounter_v1.json')
base, base_sha = load('config/tests/g1_traffic_v1.json')
grid, grid_sha = load('config/tests/e3_m3_right_grid_v1.json')
run1 = json.loads((HERE/'run1_left/report.json').read_text(encoding='utf-8'))

overlay_diff = differences(left, right)
config_diff = differences(base, grid)
allowed_overlay = {'/additional_actors[0]/lateral_offset_m', '/additional_actors[0]/gap_basis'}
allowed_config = {'/launch/grid_slot'}
unexpected_overlay = [k for k in overlay_diff if k not in allowed_overlay and top(k) not in META]
unexpected_config = [k for k in config_diff if k not in allowed_config and top(k) not in META]
checks = {
    'run1_used_base_config': run1['config_sha256'] == base_sha,
    'run1_used_left_overlay': run1['actor_overlay']['sha256'] == left_sha,
    'right_overlay_parent_is_left': right.get('parent_overlay', {}).get('sha256') == left_sha,
    'right_grid_parent_is_base': grid.get('parent_config', {}).get('sha256') == base_sha,
    'overlay_only_mirrored_lateral': not unexpected_overlay
        and overlay_diff.get('/additional_actors[0]/lateral_offset_m') == [0.13, -0.13],
    'config_only_grid_slot': not unexpected_config and config_diff.get('/launch/grid_slot') == [3, 0],
}
report = {'scope': 'configuration identity check before run2; not a driving result',
          'sha256': {'left_overlay': left_sha, 'right_overlay': right_sha, 'base_config': base_sha,
                     'right_grid_config': grid_sha},
          'overlay_differences': overlay_diff, 'config_differences': config_diff,
          'unexpected_overlay': unexpected_overlay, 'unexpected_config': unexpected_config,
          'checks': checks, 'passed': all(checks.values())}
(HERE/'run2_config_check.json').write_text(json.dumps(report, ensure_ascii=False, indent=1)+'\n',
                                           encoding='utf-8', newline='\n')
print(json.dumps({'passed': report['passed'], 'checks': checks}, ensure_ascii=False))
sys.exit(0 if report['passed'] else 1)
