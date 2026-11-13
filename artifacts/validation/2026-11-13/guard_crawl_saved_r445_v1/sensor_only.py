"""원 scan/history/기록된 path만 재구성한다. plan/후보/selector 상태 재생 없음."""
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
from arena_autonomy.numeric_kernels import configure
configure('python')
from arena_autonomy import pass_release_trial as ps
from arena_autonomy import m3_execution_trial as mx
from arena_autonomy.follow_trace import freeze, restore_call, MotionConfig, FollowConfig
from arena_autonomy.scene_motion import motion_scene

B = Path(__file__).resolve().parent
S = B / 'stage_v3'
O = B / 'sensor_only_v1'
O.mkdir(exist_ok=False)
rows = [json.loads(line) for line in (S / 'inputs/follow_decisions.jsonl').open(encoding='utf8')]
header = rows[0]['data']
cfg, motion_cfg = FollowConfig(**header['follow_config']), MotionConfig(**header['motion_config'])
# 준비된 센서 추출 함수만 재사용한다. replay_guard 모듈은 import/실행하지 않는다.
syntax = ast.parse((S / 'replay_guard.py').read_text(encoding='utf8'))
function = next(n for n in syntax.body if isinstance(n, ast.FunctionDef) and n.name == 'sensor_geometry')
exec(compile(ast.Module(body=[function], type_ignores=[]), 'frozen_sensor_geometry_only', 'exec'))
checks = []
for seq in (895, 998):
    row = rows[seq]
    assert row['seq'] == seq and row['kind'] == 'call'
    data = row['data']; normal = data['output'][2]['normal']
    info, scan, history = restore_call(data['input'], motion_cfg)
    scene = motion_scene(scan, history, info['reference_s'], timing_verified=info['timing_verified'],
                         epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'],
                         timing_policy=info.get('timing_policy', 'verified_only'))
    lat = ps.lateral_config(cfg)
    view = ps.threat_view(scene, normal['path_points'], lat)
    sides = {}
    for name in ('left', 'right'):
        expected = {k:normal['selector_evidence'][name][k] for k in ('present','points','rearmost_ds_m','frontmost_ds_m')}
        actual = view[name]
        deltas = {k:(actual[k]-expected[k]) for k in ('rearmost_ds_m','frontmost_ds_m') if actual[k] is not None and expected[k] is not None}
        count_equal = all(actual[k] == expected[k] for k in ('present','points'))
        finite_equal = all(abs(v) <= 1e-12 for v in deltas.values())
        sides[name] = dict(expected=expected, actual=actual, counts_exact=count_equal, position_delta_m=deltas,
                           position_numeric_equal_1e_minus12=finite_equal)
        assert count_equal and finite_equal
    args = (scene, view, (data['output'][0], data['output'][1], normal), (0, info['previous_steering_rad'], {}),
            info['speed_mps'], info['previous_steering_rad'], info['dt_s'], scene.t_ref_s-scene.source_first_s, {}, scan)
    geometry = sensor_geometry(SimpleNamespace(lat=lat), args)
    geometry.update(trace_seq=seq, evidence_checks=sides, source_input_sha256=hashlib.sha256(
        json.dumps(data['input'],sort_keys=True,separators=(',',':')).encode()).hexdigest(),
        source_path_sha256=hashlib.sha256(json.dumps(normal['path_points'],separators=(',',':')).encode()).hexdigest(),
        numeric_execution='python', platform='Windows sensor geometry only; no wall-time benchmark',
        full_plan_reconstructed=False, candidate_evaluations=0, source_reference_s=info['reference_s'])
    (O / f'seq{seq}.json').write_text(json.dumps(geometry,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    checks.append(dict(seq=seq,sides=sides,config=geometry['config'],threat_points=geometry['threat_points']))
paths = {name:str(Path(module.__file__).resolve()) for name,module in sys.modules.items()
         if name.startswith('arena_autonomy') and getattr(module,'__file__',None)}
assert all(str(S / 'src/arena_autonomy') in path for path in paths.values())
summary=dict(passed=True,checks=checks,module_paths=paths,full_plan_reconstructed=False,candidate_evaluations=0,
    actor_truth_used=False,source_or_policy_changed=False,numeric_mode='python',
    scope='원 evidence 점수 exact 및 투영 위치 1e-12 m 수치 대조. 실제 제어/성능/후보 수용 검증 아님.')
(O/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in summary.items() if k not in ('checks','module_paths')},ensure_ascii=False))
for c in checks:
    print(json.dumps(dict(seq=c['seq'],config=c['config'],sides={k:v['actual'] for k,v in c['sides'].items()},
        threats={k:[{kk:p[kk] for kk in ('source_index','xy','path_ds_m','path_lateral_m','expected_wall_offset_distance_m','compact_component','neighbor_line_rms_m')} for p in v] for k,v in c['threat_points'].items()}),ensure_ascii=False))
