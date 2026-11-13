"""저장 두 행의 관측 지원 호출 크기만 기록. 운영 코드 변경 없음."""
from pathlib import Path
import json,pickle,sys,time
import numpy as np
import state_codec
A=Path(__file__).resolve().parent
sys.path[:0]=[str(A/'base/src/arena_autonomy'),str(A/'base/src/arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy import observation_support as support,numeric_kernels
data=json.loads((A/'profile_inputs.json').read_bytes())
expected={tuple(x['key']):x['output'] for x in data['expected_arm']}
out=A/'diag_results';out.mkdir()
original=support.depth_support;kernel=numeric_kernels.support_classes
calls=[];kernels=[];active=None
def wrapped(scene,queries,padding,*,cycle=None,_masked_out=None):
    q=np.asarray(queries,dtype=float).reshape(-1,2)
    origins=np.array([r.ray_origin_xy_m for r in scene.rays]);directions=np.array([r.direction_xy for r in scene.rays])
    center=origins.mean(axis=0);radius=np.linalg.norm(origins-center,axis=1).max()+1e-12
    theta=np.mod(np.arctan2(directions[:,1],directions[:,0]),np.pi*2);theta=np.sort(theta,kind='stable')
    bearings=np.concatenate((theta-np.pi*2,theta,theta+np.pi*2))
    delta=q-center;distance=np.linalg.norm(delta,axis=1);window=np.isfinite(distance)&(distance>radius+1e-9)
    phi=np.mod(np.arctan2(delta[window,1],delta[window,0]),np.pi*2)
    half=np.arcsin(np.minimum(1.,radius/distance[window]))+.025+2e-10
    widths=np.searchsorted(bearings,phi+half,side='right')-np.searchsorted(bearings,phi-half,side='left')
    name=f'{active}_{len(calls)}'
    (out/(name+'.pickle')).write_bytes(pickle.dumps((scene,q,padding,cycle,_masked_out is not None),protocol=5))
    before=len(kernels);start=time.perf_counter_ns()
    answer=original(scene,queries,padding,cycle=cycle,_masked_out=_masked_out)
    calls.append(dict(name=name,key=active,queries=len(q),rays=len(origins),fallback=int((~window).sum()),
        widths={str(int(v)):int(c) for v,c in zip(*np.unique(widths,return_counts=True))},
        elapsed_ms=(time.perf_counter_ns()-start)/1e6,kernel_start=before,kernel_end=len(kernels)))
    return answer
def wrapped_kernel(part,chosen,*args):
    t=time.perf_counter_ns();value=kernel(part,chosen,*args)
    kernels.append(dict(rows=len(part),width=chosen.shape[1],real_candidates=int((chosen<len(args[0])).sum()),
        cells=chosen.size,ms=(time.perf_counter_ns()-t)/1e6))
    return value
support.depth_support=wrapped;numeric_kernels.support_classes=wrapped_kernel
plan=Evaluator('plan',data['header']);checks=[]
try:
    for item in data['selected']:
        key=tuple(item['key'])
        if key[1] not in (229,238):continue
        active=f'{key[0]}_{key[1]}'
        plan.follow=state_codec.loads((A/'states_v2'/f'{active}.pickle').read_bytes())
        output=plan(item['input']);checks.append(dict(key=list(key),raw_exact=output==expected[key]))
finally:plan.close()
result=dict(calls=calls,kernels=kernels,checks=checks)
(out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(checks=checks,calls=len(calls),kernels=len(kernels))),flush=True)
assert all(r['raw_exact'] for r in checks)
