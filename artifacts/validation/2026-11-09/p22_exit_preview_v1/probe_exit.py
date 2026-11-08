"""출구 제안과 현재 명령 검사. 평가용 자세/지도는 여기서 읽지 않는다."""
import hashlib,json,sys,math
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.p22_trial import entry_command
from exit_candidate import propose

A=Path(__file__).resolve().parent;D=A.parent/'p22_entry_wiring_v1/run1'
def main():
    p=D/'follow_decisions.jsonl';sha=lambda:hashlib.sha256(p.read_bytes()).hexdigest();before=sha()
    selected={q['t'] for q in json.loads((A/'observations.json').read_bytes())['rows']}
    rows=[]
    for l in p.open():
        e=json.loads(l);d=e['data']
        if e['kind']=='header':cfg=FollowConfig(**d['follow_config']);mc=MotionConfig(**d['motion_config'])
        if e['kind']!='call' or d['input']['reference_s'] not in selected:continue
        raw,scan,h=restore_call(d['input'],mc);t=raw['reference_s']
        scene=motion_scene(scan,h,t,timing_verified=raw['timing_verified'],epoch=raw['epoch'],rear_half_angle_deg=raw['rear_half_angle_deg'])
        paths=propose(scene,raw['previous_steering_rad'])
        for pth in paths:
            poses=np.asarray(pth['poses']);body=poses[:,:2]+cfg.wheelbase_m/2*np.column_stack((np.cos(poses[:,2]),np.sin(poses[:,2])))
            try:
                output=entry_command(scene,body,raw['speed_mps'],raw['previous_steering_rad'],raw['dt_s'],cfg)
                pth['first_output']=freeze(output)
            except Exception as exc:pth['first_error']=f'{type(exc).__name__}: {exc}'
        rows.append(dict(t=t,speed=raw['speed_mps'],paths=paths))
    out=dict(scope='sensor-derived exit geometry and first command only; no new driving',input_sha256=before,
             input_unchanged=sha()==before,rows=rows)
    (A/'exit_paths.json').write_text(json.dumps(out,separators=(',',':'))+'\n',encoding='utf8')
    print('scans',len(rows),'unique',sum(len(r['paths'])==1 for r in rows),'input_unchanged',out['input_unchanged'])
    for r in rows:
        if r['paths']:print(r['t'],[(round(p['main_wall_width_m'],3),p.get('first_output',[None,None,{}])[0:2],p.get('first_error')) for p in r['paths']])
if __name__=='__main__':main()
