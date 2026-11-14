"""Saved r550 scenes -> current helper -> current/retained geometry; not a full state replay."""
import copy
import json
import sys
from pathlib import Path
import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.current_wall_reacquire import current_wall_candidate
from arena_autonomy.current_path_recheck import _geometry as current_geometry
from arena_autonomy.race_runtime_policy.retained_recheck import _geometry as retained_geometry

P = ROOT/'artifacts/validation/2026-11-14/measured_steering_reacquire_lap_r550_v1/stall_extract'
header = json.loads((P/'header.json').read_text(encoding='utf8'))['record']['data']
owner, motion = make_controller(header)
rows = []
for name in ('last_positive', 'first_zero_candidate'):
    raw = json.loads((P/f'{name}_current_input.json').read_text(encoding='utf8'))
    data, scan, history = restore_call(raw, motion)
    scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
        epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'],
        timing_policy=data.get('timing_policy', 'verified_only'))
    candidate = current_wall_candidate(scene, data['speed_mps'], data['side'],
        data['previous_steering_rad'], data['dt_s'], owner.config)
    assert candidate is not None and 0 < candidate[0] <= .1
    output = (candidate[0], candidate[1], dict(handoff_state='FOLLOW', normal={
        **candidate[2], 'selector_mode': 'SOLO', 'selector_choice': 'baseline'}))
    before = copy.deepcopy(raw)
    checks = {key: module.rebase(output, raw, raw, scene, history, owner.config)
              for key, module in [('current', current_geometry), ('retained', retained_geometry)]}
    assert raw == before
    for key, value in checks.items():
        assert value is not None and 0 < value['speed_mps'] <= .1, (name, key, value)
        assert value['independent_guard_required'] and not value['runtime_authorized']
        assert value['source_first_s'] == scene.source_first_s
    rows.append(dict(name=name, candidate=freeze(candidate), rechecks=checks))
    print(name, 'candidate', candidate[:2], 'checks', {k:v['speed_mps'] for k,v in checks.items()})
(A/'saved_rebase.json').write_text(json.dumps(freeze(dict(
    scope='saved scenes and constructed SOLO baseline dispatch, not full planner state or driving', rows=rows)),
    indent=2, allow_nan=False), encoding='utf8')
