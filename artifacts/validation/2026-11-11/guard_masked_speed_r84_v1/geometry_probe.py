"""r83 저장 정체의 같은 조향/실측속도에서 GUARD 목표만 낮춘 영역 비교."""
import hashlib,json
from pathlib import Path
from arena_autonomy.follow_trace import restore_call,MotionConfig,FollowConfig,motion_scene
from arena_autonomy.pass_release_trial import motion_region,lateral_config,SelectorConfig,region_blocked

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
trace=ROOT/'artifacts/validation/2026-11-11/wall_occluder_r83_v1/two_actor_run1/run1/follow_decisions.jsonl'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before=sha(trace);selected=[];wanted=[39.6,40.,42.5,50.,60.]
for line in trace.open(encoding='utf-8'):
    e=json.loads(line)
    if e['kind']=='header':header=e['data']
    if e['kind']!='call' or not wanted:continue
    i=e['data']['input'];n=e['data']['output'][2].get('normal',{})
    if i['reference_s']>=wanted[0] and n.get('selector_reason')=='guard_curve_rejected_large_motion_masked_toward_left':
        wanted.pop(0);selected.append(e)
cfg=FollowConfig(**header['follow_config']);mc=MotionConfig(**header['motion_config']);rows=[]
for e in selected:
    i,scan,history=restore_call(e['data']['input'],mc)
    scene=motion_scene(scan,history,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
    n=e['data']['output'][2]['normal'];g=n['selector_guard_candidate'];attempts=[]
    for speed in (i['speed_mps'],.8):
        for target in (.35,.25,.15):
            region=motion_region(scene,speed,target,g['steering_rad'],i['previous_steering_rad'],i['dt_s'],cfg,lateral_config(cfg))
            attempts.append(dict(measured_mps=speed,target_mps=target,region=region,blocked=region_blocked(region,SelectorConfig())))
    rows.append(dict(seq=e['seq'],time=i['reference_s'],partial=n['partial_observation']['reason'],original_guard=g,attempts=attempts))
assert sha(trace)==before
out=A/'geometry_probe.json';assert not out.exists()
out.write_text(json.dumps(dict(scope='same saved geometry only; no new control permission or driving',input_sha256=before,input_unchanged=True,rows=rows),indent=2)+'\n')
print(json.dumps([dict(time=r['time'],attempts=[{k:v for k,v in a.items() if k!='region'} for a in r['attempts']]) for r in rows]))
