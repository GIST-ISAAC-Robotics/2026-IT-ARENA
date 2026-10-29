"""기록된 자세 차이를 현재 관측점/경계 기하로 분리한다. 새 주행이 아니다."""
import json
from pathlib import Path
import sys
import numpy as np

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
for folder in ('src/arena_autonomy','src/arena_vehicle_interface'):
    sys.path.insert(0,str(ROOT/folder))
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.corridor_observer import current_proposal
from arena_autonomy.corner_corridor import short_prefix_evidence
from arena_autonomy.local_path import local_path

RUNS={
    'temporary':('traffic_side_scope_v1',[113.44,113.64,113.74,113.84,114.04,114.24]),
    'official':('official_traffic_v1',[117.6,117.8,118.0,119.0,120.0,130.0]),
}


def probe(name, folder, targets):
    path=BASE.parent/folder/'run1_course/follow_decisions.jsonl'
    wanted=list(targets); rows=[]
    with path.open(encoding='utf-8') as stream:
        header=json.loads(next(stream))['data']
        config=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
        for line in stream:
            event=json.loads(line)
            if event['kind']!='call':continue
            raw=event['data'];t=raw['input']['reference_s']
            if not wanted:break
            if t+1e-9<wanted[0]:continue
            target=wanted.pop(0)
            data,scan,history=restore_call(raw['input'],motion)
            scene=motion_scene(scan,history,t,timing_verified=data['timing_verified'],epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
            proposal=current_proposal(scene,data['speed_mps'],data['side'],data['previous_steering_rad'],data['dt_s'],config)
            points=np.asarray(scene.occupied_endpoints_xy_m,dtype=float).reshape(-1,2)
            short=short_prefix_evidence(points,data['side'],config)
            paths={}
            for side in ('left','right'):
                value,info=local_path(points,side,config.wall_offset_m)
                paths[side]=dict(path=None if value is None else value.tolist(),info=info)
            row=dict(seq=event['seq'],target_s=target,t=t,speed=data['speed_mps'],side=data['side'],
                steering=data['previous_steering_rad'],recorded=raw['output'],proposal=proposal,
                short=short,paths=paths,points=points.tolist())
            rows.append(freeze(row))
    return dict(config=freeze(header['follow_config']),rows=rows)


if __name__=='__main__':
    result={name:probe(name,*values) for name,values in RUNS.items()}
    with (BASE/'geometry.json').open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
    for name,value in result.items():
        for row in value['rows']:
            print(name,json.dumps(dict(t=row['t'],reason=row['proposal']['reason'],geometry=row['proposal'].get('geometry',{}).get('reason'),
                short=row['short']['reason'],left=row['paths']['left']['info'],right=row['paths']['right']['info']),ensure_ascii=False))
