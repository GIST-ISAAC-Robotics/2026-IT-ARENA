"""가까운 첫 우회에서 현재 통로 중앙까지의 제한 편이 비교. 구동 허가는 아니다."""
from dataclasses import dataclass,replace
import math
import numpy as np
from arena_autonomy.bypass_entry_trial import (EntryConfig,entry_path,preview_clearance,
    preview_point_gap,current_outer_wall_padding)

@dataclass(frozen=True)
class AdaptiveCorridorEntryConfig(EntryConfig):
    @property
    def maximum_offset_m(self):
        return .35


def compare_roomy_path(scene,config,entry,row,wall,reuse_s):
    result=dict(applied=False,reason='no_larger_observed_center',permission_granted=False,
                unknown_filled=False,offset_m=row['offset_m'],transition_m=row['transition_m'])
    if (row['offset_m']==0 or row['transition_limit_m']>=min(entry.transitions_m)
            or not 0<=reuse_s<=.1):
        return result
    base=abs(row['offset_m'])
    # 차체 중심의 목표를 도로 반폭 이내로 제한한다. 두 바퀴 잔디 허용과
    # 관측 벽/점유 검사는 별개이며, 이 기하만으로 실제 바퀴 노면을 인증하지 않는다.
    limit=min(config.road_half_m,abs(row['want_offset_m']),entry.maximum_offset_m)
    if limit<=base+1e-9:return result
    reference=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
    if reference[3]+config.wheelbase_m/2<row['transition_m']:
        return dict(result,reason='incomplete_observed_transition')
    points=np.asarray(scene.occupied_endpoints_xy_m)
    local=replace(entry,transitions_m=(row['transition_m'],))
    # 실제 명령이 이미 사용하는 긴 현재 외벽 가정만 똑같이 적용한다.
    # 안쪽 면에는 여기서 안정/정지 가정을 새로 추가하지 않는다.
    now_pad,proof=current_outer_wall_padding(scene,config,row['side'],wall)
    aged_pad,_=current_outer_wall_padding(scene,config,row['side'],wall,growth_s=reuse_s)
    sign=math.copysign(1.,row['offset_m']);rows=[]
    for magnitude in (base,(base+limit)/2,limit):
        offset=sign*magnitude
        path=entry_path(reference,offset,config,local,row['transition_m'])
        d=np.gradient(path,axis=0);dd=np.gradient(d,axis=0)
        k=np.abs(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/np.maximum(1e-8,np.linalg.norm(d,axis=1)**3)
        steering=float(np.max(np.arctan(config.wheelbase_m*k)))
        rows.append(dict(offset_m=offset,maximum_steering_rad=steering,
            current_progress_m=preview_clearance(path,points,now_pad,config),
            current_gap_m=preview_point_gap(path,points,now_pad,config),
            reused_progress_m=preview_clearance(path,points,aged_pad,config),
            reused_gap_m=preview_point_gap(path,points,aged_pad,config)))
    def rank(r):
        return tuple(math.floor(r[k]/step) for k,step in
            [('reused_progress_m',.02),('reused_gap_m',.005),('current_progress_m',.02),('current_gap_m',.005)])
    valid=[r for r in rows if r['maximum_steering_rad']<=config.max_steering_rad
           and r['current_progress_m']>=rows[0]['current_progress_m']-.02
           and r['current_gap_m']>=0]
    best=max(valid,key=lambda r:(rank(r),-abs(r['offset_m']))) if valid else rows[0]
    applied=best['offset_m']!=row['offset_m'] and rank(best)>rank(rows[0])
    return dict(result,applied=applied,reason='current_corridor_reuse_candidate' if applied else 'no_valid_gain',
        offset_m=best['offset_m'] if applied else row['offset_m'],comparison=rows,reuse_s=reuse_s,
        existing_fixed_outer_returns=proof['static_return_count'],target_offset_limit_m=limit,
        actual_wheel_surface_verified=False)
