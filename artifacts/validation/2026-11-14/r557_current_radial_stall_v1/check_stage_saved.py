"""Explicit staged modules, captured r553 plan/current inputs; no source/ROS/device mutation."""
import importlib
import json
import sys
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.race_runtime_policy import current_route_priority

for suffix in ('current_wall_reacquire', 'current_path_recheck._geometry',
               'race_runtime_policy.retained_recheck._geometry'):
    module = importlib.import_module('arena_autonomy.'+suffix)
    path = A/'stage/src/arena_autonomy/arena_autonomy'/Path(suffix.replace('.', '/')+'.py')
    module.__file__ = str(path)
    exec(compile(path.read_text(encoding='utf8'), str(path), 'exec'), module.__dict__)
    assert Path(module.__file__).resolve() == path.resolve()
helper = importlib.import_module('arena_autonomy.current_wall_reacquire')
P = ROOT/'artifacts/validation/2026-11-14/measured_steering_radial_lap_r553_v1/stall_extract'
header = json.loads((P/'header.json').read_text(encoding='utf8'))['record']['data']
owner, motion = make_controller(header)
rows = []
for name in ('first_zero_publication_call',):
    call = json.loads((P/f'{name}_call.json').read_text(encoding='utf8'))['record']['data']
    plan = call['current_rebase']['compute']['plan']
    submitted = plan['input']
    current = call['current_rebase']['input']
    _, scan, history = restore_call(submitted, motion)
    scene = motion_scene(scan, history, submitted['reference_s'], timing_verified=submitted['timing_verified'],
        epoch=submitted['epoch'], rear_half_angle_deg=submitted['rear_half_angle_deg'],
        timing_policy=submitted.get('timing_policy', 'verified_only'))
    candidate = helper.current_wall_candidate(scene, submitted['speed_mps'], submitted['side'],
        submitted['previous_steering_rad'], submitted['dt_s'], owner.config)
    assert candidate is not None and 0 < candidate[0] <= .1
    previous = plan['output']
    output = (candidate[0], candidate[1], {**previous[2], 'normal': {
        **previous[2]['normal'], **candidate[2]}})
    _, scan, history = restore_call(current, motion)
    current_scene = motion_scene(scan, history, current['reference_s'], timing_verified=current['timing_verified'],
        epoch=current['epoch'], rear_half_angle_deg=current['rear_half_angle_deg'],
        timing_policy=current.get('timing_policy', 'verified_only'))
    result = current_route_priority.rebase(output, submitted, current, current_scene, history, owner.config)
    assert result is not None and 0 < result['speed_mps'] <= .1
    rows.append(dict(name=name, old_recorded_recheck=call['current_rebase']['output'],
        candidate=candidate, selected_recheck=result))
    print(name, candidate[:2], result['speed_mps'], result['clearance_m'])
(A/'stage_saved_result.json').write_text(json.dumps(freeze(dict(
    scope='captured r553 compute/current inputs; explicitly staged helper and two geometry modules',rows=rows)),
    indent=2, allow_nan=False), encoding='utf8')
