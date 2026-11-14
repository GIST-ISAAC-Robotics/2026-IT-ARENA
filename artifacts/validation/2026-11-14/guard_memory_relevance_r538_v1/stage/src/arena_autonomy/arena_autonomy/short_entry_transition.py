"""너무 가까운 최초 우회: 등록 전이 사이 한 중간값만 형상으로 비교한다.

센서 기반 후보 순위이며 이동 허가가 아니다. 기존 점유/관측/조향 명령 검사를
통과해야 하며 기존 경로를 이동 중 다시 바꾸지 않는다. 구동 연결 없음.
"""
from dataclasses import replace
import math
import numpy as np
from arena_autonomy.bypass_entry_trial import entry_path,preview_clearance,preview_point_gap
from arena_autonomy.occupancy_timing import per_ray_padding

def compare_short_transition(scene,config,entry,row,wall):
    base=row['transition_m'];minimum=min(entry.transitions_m)
    result=dict(applied=False,reason='ordinary_entry_distance',base_transition_m=base)
    if row['transition_limit_m']>=minimum or base<=minimum or row['offset_m']==0:
        return result
    middle=(base+minimum)/2
    reference=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
    if reference[3]+config.wheelbase_m/2<base:
        return dict(result,reason='incomplete_observed_transition')
    local=replace(entry,transitions_m=(middle,base))
    points=np.asarray(scene.occupied_endpoints_xy_m)
    padding=per_ray_padding(scene,.025+config.measurement_padding_m,config.object_motion_bound_mps)
    rows=[]
    for transition in (middle,base):
        path=entry_path(reference,row['offset_m'],config,local,transition)
        d=np.gradient(path,axis=0);dd=np.gradient(d,axis=0)
        curvature=np.abs(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/np.maximum(1e-8,np.linalg.norm(d,axis=1)**3)
        maximum=float(np.max(np.arctan(config.wheelbase_m*curvature)))
        rows.append(dict(transition_m=transition,maximum_steering_rad=maximum,
            progress_m=preview_clearance(path,points,padding,config),
            point_gap_m=preview_point_gap(path,points,padding,config)))
    new,old=rows
    # 기존 EntryGuide와 같은 20 mm 진행/5 mm 간격 순위를 쓴다. 별도의
    # 20 mm 간격 개선 문턱은 기존 선택 규칙에 없으므로 적용하지 않는다.
    rank=lambda r:(math.floor(r['progress_m']/.02),math.floor(r['point_gap_m']/.005))
    valid=(new['maximum_steering_rad']<=config.max_steering_rad
           and rank(new)>rank(old))
    return dict(result,applied=valid,reason='earlier_observed_path_candidate' if valid else 'no_valid_clearance_gain',
                transition_m=middle if valid else base,comparison=rows,
                ranking='existing_20mm_progress_5mm_point_gap',
                permission_granted=False,unknown_filled=False)
