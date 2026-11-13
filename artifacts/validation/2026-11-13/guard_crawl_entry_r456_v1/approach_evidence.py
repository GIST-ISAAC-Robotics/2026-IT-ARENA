"""복제된 실제 첫 정체 입력의 거절 표본을 분류한다. 판정 함수는 바꾸지 않는다."""
import copy
import hashlib
import json
from pathlib import Path
import pickle
import sys
import numpy as np

B=Path(__file__).resolve().parent
sys.path[:0]=[str(B/'base/src/arena_autonomy'),str(B/'base/src/arena_vehicle_interface')]
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.follow_trace import freeze
from arena_autonomy.numeric_kernels import configure
configure('native_v1')
snapshots=pickle.loads((B/'pinpoint_v1/local_actual_snapshots.pkl').read_bytes())
identity,snapshot=snapshots[0]
owner,scene,baseline,selected,speed,previous,dt,raw_scan,measured=copy.deepcopy(snapshot)
view=ps.threat_view(scene,baseline[2].get('path_points'),owner.lat)
grid=ps.mx.ReturnGrid(scene,owner.lat)
ds,masks=ps.mx.side_return_masks(view['frame'],view['state'],grid,owner.lat)
threats={side:dict(points=grid.returns[mask].tolist(),ds_m=ds[mask].tolist()) for side,mask in zip(('left','right'),masks)}
original=ps.observed_guard_approach
captures=[]

def observe(frame,context,guarded,rejected):
    result=original(frame,context,guarded,rejected)
    _,lateral=frame.project(context['queries'])
    details={}
    for side in rejected:
        sign=1 if side=='left' else -1
        beyond=sign*lateral>guarded[side]['current_m']+1e-9
        missing=beyond & ~context['free']
        masked=context.get('masked',np.zeros(len(beyond),dtype=bool))
        rows=context['queries'][missing]
        details[side]=dict(samples=int(beyond.sum()),observed=int((beyond & context['free']).sum()),
            missing=int(missing.sum()),missing_masked=int((missing & masked).sum()),
            missing_nonmasked=int((missing & ~masked).sum()),
            missing_query_bounds_xy_m=[rows.min(axis=0).tolist(),rows.max(axis=0).tolist()],
            missing_queries_xy_m=rows.tolist())
    captures.append(dict(target_cap=cap,original_result=result,classification=details))
    return result

results=[]
before=hashlib.sha256(pickle.dumps(owner,protocol=5)).hexdigest()
ps.observed_guard_approach=observe
try:
    for cap in (.15,.1):
        parallel=(selected[0],selected[1],dict(selected[2],reason=selected[2]['selector_guard_candidate']['trigger']))
        checked=owner._guard_curve_at_cap(scene,view,baseline,parallel,speed,previous,dt,
            scene.t_ref_s-scene.source_first_s,{},raw_scan,target_cap=cap)
        proof=checked[2]['selector_guard_candidate']
        results.append(dict(cap=cap,command=checked[:2],reason=checked[2]['selector_reason'],
            proof=proof,region_blocked=ps.region_blocked(proof['region'],owner.config)))
finally:ps.observed_guard_approach=original
assert hashlib.sha256(pickle.dumps(owner,protocol=5)).hexdigest()==before
output=dict(identity={k:identity[k] for k in ('generation','sequence','reference_s','measured')},
    source_first_s=scene.source_first_s,source_epoch=scene.source_epoch,baseline_speed=baseline[0],
    baseline_reason=baseline[2]['reason'],baseline_swept_clearance_m=baseline[2].get('swept_clearance_m'),
    selector_sides=freeze(owner.sides),guard_anchor=freeze(owner.guard_anchor),
    sensor_threat_returns=threats,results=freeze(results),captures=freeze(captures),
    interpretation='all recorded missing extra-approach samples classified as installation/self-mask; not observed finite collision evidence; hidden motion risk not excluded',
    state_unchanged=True,thresholds_unchanged=True,controller_truth_used=False)
(B/'pinpoint_v1/approach_geometry.json').write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
compact=copy.deepcopy(output)
for capture in compact['captures']:
    for d in capture['classification'].values():d.pop('missing_queries_xy_m')
print(json.dumps(compact,ensure_ascii=False))
