"""r50 저장 입력에서 출구 전환 때 유지할 입구 경로만 대조한다. 새 주행 아님."""
import hashlib,json,math
from pathlib import Path
from unittest.mock import patch
import numpy as np
from arena_autonomy.follow_trace import replay_call,restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.p22_trial import P22Entry,entry_command
from arena_autonomy.p22_route_trial import P22Route,body_path
from arena_autonomy.scene_motion import motion_scene
from extend_path import propose

A=Path(__file__).resolve().parent
source=A.parent/'p22_route_connected_v1/run1/follow_decisions.jsonl'
sha=lambda:hashlib.sha256(source.read_bytes()).hexdigest()
before=sha();rows=[];first=None;original_transition=None
for line in source.open():
    e=json.loads(line);k=e['kind'];d=e['data']
    if k=='header':
        mc=MotionConfig(**d['motion_config']);cfg=FollowConfig(**d['follow_config'])
        new=P22Route(LocalFollow(cfg));base=P22Entry(LocalFollow(cfg))
    elif k=='p22_frame':new.frame(**d);base.frame(**d)
    elif k=='reset':
        force=d.get('force',d['reason'] in ('autonomy_reset','legacy_reset'));new.reset(force=force);base.reset(force=force)
    elif k=='call':
        raw,scan,h=restore_call(d['input'],mc);t=raw['reference_s']
        # 원 입구 경로만 계속 이송하여 같은 프레임의 제안을 비교한다.
        replay_call(base,d['input'],mc)
        if first is None:
            with patch('arena_autonomy.p22_route_trial.propose',side_effect=lambda s,p:propose(s,p,new.path)):
                out=replay_call(new,d['input'],mc)
            if new.phase=='exit':
                first=dict(t=t,output=freeze(out),path=new.path.tolist())
        if d['output'][2].get('p22_trial',{}).get('phase')=='exit' and original_transition is None:original_transition=t
        if original_transition is not None and abs(t-original_transition)<1e-7:
            scene=motion_scene(scan,h,t,timing_verified=raw['timing_verified'],epoch=raw['epoch'],rear_half_angle_deg=raw['rear_half_angle_deg'])
            candidates=propose(scene,raw['previous_steering_rad'],base.path)
            for p in candidates:
                path=body_path(p,cfg.wheelbase_m)
                output=entry_command(scene,path,raw['speed_mps'],raw['previous_steering_rad'],raw['dt_s'],cfg)
                rows.append(dict(t=t,retained_max_body_difference_m=float(np.max(np.linalg.norm(path[:len(base.path)]-base.path,axis=1))),
                    candidate=p,output=freeze(output),original_output=d['output'][:2]))
out=dict(scope='retained-entry path first-command comparison only; not new driving',input_unchanged=sha()==before,
         input_sha256=before,original_transition_s=original_transition,new_first_exit=first,rows=rows)
(A/'probe_summary.json').write_text(json.dumps(out,separators=(',',':'))+'\n')
print('original',original_transition,'new_first',None if first is None else (first['t'],first['output'][:2]))
for r in rows:print('candidate',r['t'],r['retained_max_body_difference_m'],r['output'][:2],r['candidate']['retained_offset_m'])
assert out['input_unchanged'] and first and first['output'][0]>0
