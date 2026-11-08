"""Read-only observation audit behind GUARD's lateral-anchor rejection."""
from pathlib import Path
import hashlib,importlib.util,json,sys
from collections import Counter
import numpy as np
from arena_autonomy.follow_trace import FollowConfig,MotionConfig,replay_call,freeze
from arena_autonomy.local_follow import LocalFollow
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
import arena_autonomy.pass_release_trial as pr

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
P=A.parent/'traffic_corner_prefix_probe_v1'
spec=importlib.util.spec_from_file_location('detached_prefix_for_guard',P/'candidate_local_follow.py')
m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)
LocalFollow._baseline_command=m.LocalFollow._baseline_command
original=pr.PassReleaseSelector._guard_curve
source={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
captures=[];sources=set();counts=Counter()

def guarded(self,scene,view,baseline,parallel,speed,previous,dt,age,detail,raw_scan):
    out=original(self,scene,view,baseline,parallel,speed,previous,dt,age,detail,raw_scan)
    if out is None or 'guard_curve_rejected_toward_' not in out[2].get('selector_reason',''):return out
    counts['toward_calls']+=1
    if scene.source_first_s in sources:return out
    sources.add(scene.source_first_s)
    proof=out[2]['selector_guard_candidate'];target=proof['target_speed_mps'];context={}
    region=pr.motion_region(scene,speed,target,baseline[1],previous,dt,self.follow_config,self.lat,context=context)
    if not context:
        _,q=pr.motion_queries(scene,speed,target,baseline[1],previous,dt,self.follow_config,self.lat)
        pad=pr.mx.ray_padding(scene,self.lat,self.follow_config.object_motion_bound_mps)
        free=pr.depth_support(scene,q,pad)
        context=dict(queries=q,free=free,masked=np.zeros(len(q),bool),padding=pad)
    q=context['queries'];_,l=view['frame'].project(q)
    toward={}
    for name,sign in pr.SIDES:
        d=proof['guarded_sides'].get(name)
        if not d:continue
        if not(d['predicted_m']>d['anchor_m']+self.config.small_excursion_m and d['predicted_m']>d['current_m']+1e-9):continue
        mask=sign*l>d['current_m']+1e-9
        toward[name]=dict(samples=int(mask.sum()),observed=int(context['free'][mask].sum()),
            masked=int(context['masked'][mask].sum()),all_observed=bool(mask.any() and context['free'][mask].all()))
    row=dict(t=scene.t_ref_s,source=scene.source_first_s,speed=speed,baseline=list(baseline[:2]),
        old=out[:2],reason=out[2]['selector_reason'],guarded_sides=proof['guarded_sides'],toward=toward,region=region,
        fresh_path=baseline[2].get('path_source_s')==scene.source_first_s and not baseline[2].get('boundary_from_memory'),
        whole_region_block=pr.region_blocked(region,self.config),current_only=bool(toward and all(v['all_observed'] for v in toward.values())))
    captures.append(row)
    return out

pr.PassReleaseSelector._guard_curve=guarded
trace=A.parent/'corner_slowing_actor_v1/run1/follow_decisions.jsonl'
expected=[json.loads(l)['new'] for l in (P/'replay_rows.jsonl').read_text().splitlines()]
calls=bad=0
for line in trace.open():
    e=json.loads(line);d=e['data']
    if e['kind']=='header':
        c=BypassHandoffFollow(FollowConfig(**d['follow_config']),HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
        mc=MotionConfig(**d['motion_config'])
    elif e['kind']=='reset':c.reset()
    elif e['kind']=='call':
        out=freeze(replay_call(c,d['input'],mc));bad+=not np.allclose(out[:2],expected[calls],atol=1e-10,rtol=0);calls+=1
assert source=={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
summary=dict(calls=calls,detached_prefix_mismatch=bad,counts=counts,unique_sources=len(captures),
    current_only=sum(r['current_only'] for r in captures),current_only_and_region_clear=sum(r['current_only'] and r['whole_region_block'] is None for r in captures),source_unchanged=True)
(A/'captures.json').write_text(json.dumps(freeze(captures),indent=2))
(A/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary));assert bad==0
