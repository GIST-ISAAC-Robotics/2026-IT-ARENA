"""후방 현재 면의 거부 위치와 r44 통로 예산을 저장 입력에서 분해한다."""
from collections import Counter
from pathlib import Path
import hashlib
import json
import math
import numpy as np
from arena_autonomy.follow_trace import restore_call, motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.known_structure import classify, parse_sectors, ACCEPTED_HIDDEN_WIDTH_M
from arena_autonomy.wall_reference_support import observed_rear_wall_face

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
R32=ROOT/'artifacts/validation/2026-11-08/moving_pass_wall_mask_v1/straight_moving_lead/run1/follow_decisions.jsonl'
R44=ROOT/'artifacts/validation/2026-11-08/faster_entry_consistency_r44_v1/straight_moving_lead/run1/follow_decisions.jsonl'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def explain(scene,cfg,side,wall,explained):
    """원 함수의 수치는 유지하고 최초 거부 단계만 설명한다."""
    a,b=wall['slope'],wall['intercept_m']
    lower,upper=wall['observed_x_m']; minimum=math.hypot(cfg.length_m,cfg.width_m)+2*cfg.measurement_padding_m+.03
    proof={'minimum_rear_span_m':minimum,'main_bounds':[lower,upper]}
    if wall.get('reason')!='current_connected_straight_wall_fit':return {**proof,'reason':'fit_reason'}
    if upper<.30 or upper-lower<.65:return {**proof,'reason':'principal_length'}
    ids=wall['source_ray_indices'];rays={r.source_index:r for r in scene.rays}
    edge,step=min(((ids[0],-1),(ids[-1],1)),key=lambda e:rays[e[0]].hit_xy_m[0])
    i=edge+step;gap=[]
    while i in rays and rays[i].status=='masked':gap.append(i);i+=step
    proof.update(edge=edge,mask=gap,next_index=i,next_status=rays[i].status if i in rays else None)
    if not gap or frozenset(gap) not in explained or i not in rays or rays[i].status!='return':
        return {**proof,'reason':'mask_not_adjacent_explained_return'}
    chord=math.dist(rays[edge].hit_xy_m,rays[i].hit_xy_m);proof['chord_m']=chord
    if chord>ACCEPTED_HIDDEN_WIDTH_M:return {**proof,'reason':'shadow_chord'}
    norm=math.hypot(1.,a);sign=1 if side=='left' else -1
    actual_b=b+sign*cfg.wall_offset_m*norm;shift=sign*cfg.wall_offset_m*a/norm
    points=[];last=rays[edge].hit_xy_m;stop='ray_status'
    while i in rays and rays[i].status=='return':
        p=rays[i].hit_xy_m;distance=math.dist(last,p);residual=abs(p[1]-a*p[0]-actual_b)/norm
        if not -1.5<=p[0]<last[0]:stop='monotonic_or_rear_limit';break
        if distance>.05:stop='pair_distance';break
        if residual>.015:stop='line_residual';break
        points.append(p);last=p;i+=step
    proof.update(points=len(points),walk_end_reason=stop,end_index=i)
    if i in rays and rays[i].status=='return':proof.update(end_pair_distance_m=distance,end_residual_m=residual,end_point=list(p))
    span=points[0][0]-points[-1][0] if points else 0.;proof['rear_span_m']=span
    if len(points)<12 or span<minimum:return {**proof,'reason':'rear_count_or_span'}
    x,y=np.asarray(points).T
    slope=np.dot(x-x.mean(),y-y.mean())/np.dot(x-x.mean(),x-x.mean())
    new_lower=float(min(x)+shift);angle=abs(math.atan(slope)-math.atan(a))
    proof.update(new_lower_m=new_lower,slope_angle_rad=float(angle))
    if new_lower>-cfg.wheelbase_m/2 or angle>.08:return {**proof,'reason':'rear_axis_or_angle'}
    return {**proof,'reason':'accepted'}


hashes={str(p.relative_to(ROOT)):sha(p) for p in (R32,R44)}
rows=[];seen=set();counts=Counter()
for line in R32.open():
    e=json.loads(line);d=e['data']
    if e['kind']=='header':cfg=FollowConfig(**d['follow_config']);mc=MotionConfig(**d['motion_config'])
    if e['kind']!='call' or d['output'][0]>0:continue
    entry=d['output'][2].get('moving_pass',{}).get('entry',{})
    if not entry.get('candidates'):continue
    row=entry['candidates'][0];side=row.get('current_outer_wall')
    if side not in ('left','right') or not entry.get('reference',{}).get('walls',{}).get(side):continue
    original,scan,history=restore_call(d['input'],mc)
    scene=motion_scene(scan,history,original['reference_s'],timing_verified=original['timing_verified'],epoch=original['epoch'],rear_half_angle_deg=original['rear_half_angle_deg'])
    key=(scene.source_first_s,side)
    if key in seen:continue
    seen.add(key);wall=entry['reference']['walls'][side]
    _,_,sectors=classify(scan,parse_sectors(cfg.known_structure_sectors_json))
    explained={frozenset(r['rays'].tolist()) for r in sectors if r['explained']}
    result=explain(scene,cfg,side,wall,explained)
    exact=observed_rear_wall_face(scene,cfg,side,(wall['slope'],wall['intercept_m'],*wall['observed_x_m']),wall,explained,minimum_rear_span_m=result['minimum_rear_span_m'],rear_limit_m=-1.5)
    assert (exact is not None)==(result['reason']=='accepted')
    counts[result['reason']]+=1
    rows.append(dict(t=original['reference_s'],source=scene.source_first_s,side=side,entry_reason=row['reason'],diagnostic=result))
width=[]
for line in R44.open():
    e=json.loads(line)
    if e['kind']!='call':continue
    d=e['data'];entry=d['output'][2].get('moving_pass',{}).get('entry',{})
    data=entry.get('pass_width')
    if not data:continue
    width.append(dict(t=d['input']['reference_s'],speed=d['output'][0],reason=entry.get('reason'),sides=data))
width_summary={}
for side in ('left','right'):
    values=[r['sides'][side] for r in width if side in r['sides']]
    if values:
        width_summary[side]=dict(calls=len(values),feasible=sum(v['feasible'] for v in values),
            width_range_m=[min(v['current_observed_width_m'] for v in values),max(v['current_observed_width_m'] for v in values)],
            budget_width_range_m=[min(v['high_m']-v['low_m'] for v in values),max(v['high_m']-v['low_m'] for v in values)],
            first=values[0])
summary=dict(scope='stored input diagnostics only; no new commands or trajectory',r32_distinct_stopped_scenes=len(rows),rear_rejections=counts,r44_width=width_summary,input_hashes=hashes)
for name,data in [('summary.json',summary),('rear_rows.json',rows),('width_rows.json',width)]:
    (A/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
assert all(sha(ROOT/p)==v for p,v in hashes.items())
print(json.dumps(summary,ensure_ascii=False))
