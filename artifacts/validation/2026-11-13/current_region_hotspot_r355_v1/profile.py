from pathlib import Path
import cProfile
import importlib.util
import json
import pstats
import sys

root=Path(sys.argv[1]).resolve()
sys.path[:0]=[str(root/'src/arena_autonomy'),str(root/'src/arena_vehicle_interface')]
a=root/'artifacts/validation/2026-11-13/final_geometry_current_cost_r354_v1'
spec=importlib.util.spec_from_file_location('cost',a/'measure.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
data=json.loads((a/'selected.json').read_bytes());h=data['header']
motion,cfg=m.helper.MotionConfig(**h['motion_config']),m.helper.FollowConfig(**h['follow_config'])
m.helper.configure('native_v1');results=[]
for item in data['rows']:
    if item['index'] not in (37,70,80):continue
    c=item['call'];current=c['current_rebase']['input']
    d,scan,history=m.helper.restore_call(current,motion)
    scene=m.helper.motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    profiler=cProfile.Profile();profiler.enable()
    answer=m.policy.rebase(c['output'],m.helper.plan_reference(c['input']),current,scene,history,cfg,require_current_region=True)
    profiler.disable();stats=pstats.Stats(profiler)
    rows=[]
    for (path,line,name),(cc,nc,tt,ct,_) in sorted(stats.stats.items(),key=lambda x:x[1][3],reverse=True)[:30]:
        rows.append(dict(file=path.replace(str(root)+'/',''),line=line,name=name,calls=nc,self_ms=tt*1000,cumulative_ms=ct*1000))
    results.append(dict(index=item['index'],reference_s=current['reference_s'],speed=answer['speed_mps'],reason=answer['reason'],top=rows))
out=Path(sys.argv[2]);assert not out.exists()
out.write_text(json.dumps(dict(scope='diagnostic cProfile overhead; use r354 for unprofiled cost',cases=results),indent=2)+'\n')
print(json.dumps([dict(index=r['index'],speed=r['speed'],reason=r['reason'],top=r['top'][:6]) for r in results]))
