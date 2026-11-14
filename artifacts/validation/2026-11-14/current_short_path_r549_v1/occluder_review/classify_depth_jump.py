"""저장 평가 자세와 당시 SDF를 사후 대조한다. 제어 입력/기하 생성 없음."""
from pathlib import Path
import json, math, xml.etree.ElementTree as E
import numpy as np
from check_windows import restored, A, P
_,data,scene=restored('first_zero_candidate')
run=P.parent
poses=[json.loads(line) for line in (run/'run1/trajectory.jsonl').open()]
before=max((p for p in poses if p['t']<=scene.t_ref_s),key=lambda p:p['t'])
after=min((p for p in poses if p['t']>=scene.t_ref_s),key=lambda p:p['t'])
fraction=(scene.t_ref_s-before['t'])/(after['t']-before['t'])
pose={key:before[key]+fraction*(after[key]-before[key]) for key in ('t','x','y','yaw')}
c,s=math.cos(pose['yaw']),math.sin(pose['yaw'])
rotation=np.array([[c,s],[-s,c]])
boxes=[]
tree=E.parse(run/'field_world.sdf')
for visual in tree.getroot().iter('visual'):
    size=visual.findtext('geometry/box/size')
    if not size:continue
    q=list(map(float,visual.findtext('pose','0 0 0 0 0 0').split()))
    dim=list(map(float,size.split()))
    # 현재 월드 track_geometry_merged의 wall box pose는 월드 좌표이다.
    if 'walls_' not in visual.get('name',''):continue
    boxes.append((visual.get('name'),q,dim))
rows=[]
for index in (166,167,171,172,173):
    ray=next(ray for ray in scene.rays if ray.source_index==index)
    point=np.asarray(ray.hit_xy_m)@rotation+[pose['x'],pose['y']]
    nearest=[]
    for name,q,dim in boxes:
        c,s=math.cos(q[5]),math.sin(q[5])
        local=(point-q[:2])@np.array([[c,-s],[s,c]])
        excess=np.maximum(abs(local)-np.asarray(dim[:2])/2,0.)
        distance=float(np.linalg.norm(excess))
        nearest.append(dict(name=name,point_to_box_m=distance,box_pose=q,box_size=dim,local_point=local.tolist()))
    rows.append(dict(source=index,world_point_approx=point.tolist(),nearest=sorted(nearest,key=lambda r:r['point_to_box_m'])[:2]))
output=dict(scope='evaluation only; linear interpolation between saved poses, not exact ray collider identity',reference_s=scene.t_ref_s,
    pose=pose,pose_bracket_s=[before['t'],after['t']],rows=rows)
(A/'depth_jump_evaluation.json').write_text(json.dumps(output,indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(pose_bracket_s=output['pose_bracket_s'],rows=[dict(source=r['source'],nearest=r['nearest'][0]) for r in rows]),indent=2))
