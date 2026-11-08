"""WSL PC 저장 장면의 추가 검사 비용. 전체 제어기나 Jetson 성능값 아님."""
from pathlib import Path
import json,time
from arena_autonomy.follow_trace import restore_call,motion_scene,MotionConfig,FollowConfig
from arena_autonomy.masked_motion_speed import lower_masked_motion
from arena_autonomy.pass_release_trial import SelectorConfig,motion_region,lateral_config
A=Path(__file__).resolve().parent;probe=A.parent/'released_masked_speed_probe_v1';rows=[]
for d in json.loads((probe/'snapshots.json').read_bytes()):
    cfg=FollowConfig(**d['header']['follow_config']);mc=MotionConfig(**d['header']['motion_config'])
    i,scan,h=restore_call(d['input'],mc)
    scene=motion_scene(scan,h,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
    n=d['output'][2]['normal']
    def evaluate(target):
        context={}
        r=motion_region(scene,i['speed_mps'],target,d['baseline_steering_rad'],i['previous_steering_rad'],i['dt_s'],cfg,lateral_config(cfg),context=context)
        return r,context
    samples=[]
    for k in range(3):
        start=time.perf_counter();out=lower_masked_motion(n['selector_region'],n['partial_observation'],d['baseline_target_mps'],SelectorConfig(),evaluate)
        samples.append((time.perf_counter()-start)*1000)
    rows.append(dict(t=scene.t_ref_s,extra_check_ms=samples,attempts=len(out['diagnostic']['attempts']),target_mps=out['target_mps']))
result=dict(scope='PC WSL, 3 repeats per saved scene, regression may concurrently use CPU; extra retry only, not whole controller or Jetson',rows=rows)
(A/'cost_result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
