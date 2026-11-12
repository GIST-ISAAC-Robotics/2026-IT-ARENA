"""같은 현재 검사의 중복 계산 생략 전후 전체 출력 대조. 같은 측정 범위만 비교."""
import json,sys,time,statistics
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3];B=A.parent/'opposite_reacquire_runtime_r279_v1'
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface'),str(A.parent/'retained_route_execution_r266_v1')]
from arena_autonomy.follow_trace import restore_call,equal_output,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.numeric_kernels import configure
from overlap import plan_reference
import candidate,candidate_before_fresh_reuse as before

with (B/'two_actor_run1/run1/follow_decisions.jsonl').open() as f:
    header=json.loads(next(f))['data'];calls=[e['data'] for line in f if (e:=json.loads(line))['kind']=='call' and e['data'].get('current_rebase')]
cfg=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config']);configure('native_v1')
times={'before':[],'after':[]};equal=0;checks={'before':0,'after':0};rows=[]
for label,module in [('before',before),('after',candidate)]:
    original=module.check
    def counted(*args,_original=original,_label=label,**kwargs):
        checks[_label]+=1;return _original(*args,**kwargs)
    module.check=counted
for index,c in enumerate(calls):
    current=c['current_rebase']['input'];d,scan,h=restore_call(current,motion)
    scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    args=(c['output'],plan_reference(c['input']),current,scene,h,cfg);outputs={}
    order=[('before',before),('after',candidate)]
    if index%2:order.reverse()
    for label,module in order:
        start=time.perf_counter_ns();outputs[label]=freeze(module.rebase(*args,require_current_region=True));times[label].append((time.perf_counter_ns()-start)/1e6)
    assert equal_output(outputs['before'],outputs['after']),d['reference_s']
    equal+=1
    rows.append(dict(reference_s=d['reference_s'],before_ms=times['before'][-1],after_ms=times['after'][-1]))
def stats(x):
    q=sorted(x);return dict(median=statistics.median(q),p95=q[int((len(q)-1)*.95)],maximum=max(q))
result=dict(equal=equal,extra_full_checks=checks,core_ms={k:stats(v) for k,v in times.items()},scope='PC same-scope alternating paired replay; not Jetson deadline')
(A/'reuse_comparison.json').write_text(json.dumps(result,indent=2)+'\n');(A/'reuse_rows.json').write_text(json.dumps(rows,indent=2)+'\n');print(json.dumps(result))
