"""r45 실제 입력을 현재 코드로 재현하고 경계/측방선택의 입력을 읽기 전용 계측."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
from arena_autonomy.follow_trace import FollowConfig, MotionConfig, freeze, replay_call
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.local_follow import LocalFollow, transport_points
from arena_autonomy.pass_release_trial import PassReleaseSelector

A = Path(__file__).resolve().parent; ROOT = A.parents[3]
trace = A.parent/'corner_occlusion_connected_r45_v1/run1/follow_decisions.jsonl'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
source = {str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
base_original = LocalFollow._baseline_command
select_original = PassReleaseSelector.select
captures, rows, current = [], [], {}
last_source = set()
fields = ('reason','baseline_reason','behavior','boundary_selection','boundary_candidate_reason',
    'fresh_boundary_accepted','boundary_from_memory','path_wall','wall_fallback','path_source_s',
    'observed_path_ahead_m','path_points','boundary_age_s','target_speed_before_hold_mps',
    'swept_clearance_m','current_corner_support','selector_choice','selector_reason',
    'selector_guard_candidate','selector_sides','selector_evidence','partial_observation',
    'corridor_candidate','parallel_hold_trial')

def compact(out):
    return {'speed':float(out[0]),'steer':float(out[1]),
            'detail':{k:v for k,v in out[2].items() if k in fields}}

def baseline(self, scene, history, *args, **kw):
    carried = None
    if scene.t_ref_s>=32 and self.path is not None:
        try: carried = transport_points(self.path, history, self.path_ref, scene.t_ref_s)
        except ValueError: pass
    out = base_original(self, scene, history, *args, **kw)
    if scene.t_ref_s>=32:
        current.setdefault('baselines',[]).append(compact(out))
        key = scene.source_first_s
        if key not in last_source:
            last_source.add(key)
            captures.append(dict(t=scene.t_ref_s,source=key,carried_path=carried,
                baseline=compact(out), rays=[dict(index=r.source_index,status=r.status,
                    origin=r.ray_origin_xy_m,direction=r.direction_xy,hit=r.hit_xy_m)
                    for r in scene.rays]))
    return out

def select(self, scene, base, parallel, speed, previous, dt, **kw):
    out = select_original(self, scene, base, parallel, speed, previous, dt, **kw)
    if scene.t_ref_s>=32:
        current.setdefault('selectors',[]).append(dict(t=scene.t_ref_s,source=scene.source_first_s,
            measured=speed,previous=previous,baseline=compact(base),parallel=compact(parallel),
            selected=compact(out)))
    return out

LocalFollow._baseline_command = baseline
PassReleaseSelector.select = select
counts=Counter(); mismatch=0; first=None
for line in trace.open():
    event=json.loads(line); d=event['data']; counts[event['kind']]+=1
    if event['kind']=='header':
        c=BypassHandoffFollow(FollowConfig(**d['follow_config']),
            HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
        mc=MotionConfig(**d['motion_config'])
    elif event['kind']=='reset': c.reset()
    elif event['kind']=='call':
        current={}
        out=freeze(replay_call(c,d['input'],mc))
        bad=not np.allclose(out[:2],d['output'][:2],atol=1e-10,rtol=0)
        mismatch+=bad
        if bad and first is None:first=dict(t=d['input']['reference_s'],old=d['output'][:2],new=out[:2])
        if d['input']['reference_s']>=32:
            rows.append(dict(t=d['input']['reference_s'],output=out[:2],normal=compact((out[0],out[1],out[2].get('normal',{}))),**current))
assert source=={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
(A/'captures.json').write_text(json.dumps(freeze(captures)))
(A/'calls.json').write_text(json.dumps(freeze(rows)))
summary=dict(events=counts,recorded_command_mismatch=mismatch,first_mismatch=first,
    captured_scans=len(captures),captured_calls=len(rows),trace_sha256=sha(trace),source_sha256=source)
(A/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k!='source_sha256'}))
assert mismatch==0
