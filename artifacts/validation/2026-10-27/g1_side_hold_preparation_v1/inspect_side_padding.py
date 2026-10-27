"""평행 후보를 막는 현재 반환의 위치·점별 나이·팽창 항목을 분리한 저장 진단."""
from collections import Counter
import hashlib
import json

import numpy as np

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.scene_motion import motion_scene
from probe_saved import CASES, ROOT, HERE


def one(name, relative):
    prior={r['seq']:r for r in (json.loads(l) for l in (HERE/f'run2_{name}_rows.jsonl').read_text().splitlines())
           if r['stored_sensor_threat'] and r['candidate']['reason']=='unchanged_gap_hold'}
    path=ROOT/relative
    before=hashlib.sha256(path.read_bytes()).hexdigest()
    rows=[]
    for line in path.open(encoding='utf-8'):
        e=json.loads(line)
        if e['kind']=='header':
            motion=MotionConfig(**e['data']['motion_config']);cfg=FollowConfig(**e['data']['follow_config'])
        elif e['kind']=='call' and e['seq'] in prior:
            d,scan,h=restore_call(e['data']['input'],motion)
            scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],
                epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
            points=np.asarray(scene.occupied_endpoints_xy_m)
            old=per_ray_padding(scene,.025+cfg.measurement_padding_m,.5)
            alternative=per_ray_padding(scene,.025+cfg.measurement_padding_m,.15)
            hit=(abs(points[:,0])<=.1+old)&(abs(points[:,1])<=.075+old)
            parallel=(abs(points[:,0])<=.1+old)&(abs(points[:,1])<=.075+alternative)
            ids=np.flatnonzero(hit)
            rows.append({'seq':e['seq'],'reference_s':d['reference_s'],
                'hit_at_current_pose':len(ids),'alternative_hit_at_current_pose':int(np.count_nonzero(parallel)),
                'samples':[{'xy_m':points[i].tolist(),'lateral_gap_from_body_m':float(abs(points[i,1])-.075),
                    'original_padding_m':float(old[i]),'same_base_point15_motion_m':float(alternative[i])}
                    for i in ids[:8]]})
    return {'examined_calls':len(rows),'original_current_overlap_calls':sum(r['hit_at_current_pose']>0 for r in rows),
        'alternative_current_overlap_calls':sum(r['alternative_hit_at_current_pose']>0 for r in rows),
        'input_unchanged':before==hashlib.sha256(path.read_bytes()).hexdigest(),'input_sha256':before,
        'rows':rows}


if __name__=='__main__':
    target=HERE/'run3_padding.json'
    if target.exists():raise FileExistsError(target)
    result={'scope':'current_pose_padding_diagnostic_not_swept_permission_or_new_policy',
        'comparison':'longitudinal .5 unchanged, hypothetical lateral .15 m/s age term, base .055 m unchanged',
        'basis':'existing M3 other_lateral_speed_mps=.15 is a developer assumption, not measured bound',
        'results':{name:one(name,path) for name,path in CASES.items()}}
    with target.open('x',encoding='utf-8') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:{q:v for q,v in r.items() if q!='rows'} for k,r in result['results'].items()},indent=2))
