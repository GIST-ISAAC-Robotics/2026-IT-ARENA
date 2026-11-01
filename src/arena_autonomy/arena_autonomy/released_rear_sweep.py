"""측방 통과 뒤 저속의 작은 후반부 무반환 이동. 자유 공간 증명이 아니다.

기존 통과 이력 유지 분기에서만 호출한다. 앞부분·범퍼 너머·현재 점유는 유지한다.
1 cm 영역은 공백 하나 3도, 그중 1 mm 영역만 기존 전체 공백 6도 예산을
한 공백에도 적용한다. 가려진 돌출물·차체/운동 오차의 위험은 남는다.
"""
import math
import numpy as np
from arena_autonomy.observation_support import source_valid
from arena_autonomy.rear_fringe_trial import _bounded_nonreturn


def bounded_released_rear_sweep(scene, raw_scan, queries, padding, length, width,
                               speed, target, maximum_exterior_m):
    result=dict(accepted=False, reason='invalid_input', unknown_filled=False,
        residual_hidden_object_risk=True, maximum_exterior_m=maximum_exterior_m,
        maximum_gap_rad=math.radians(3), minimum_neighbor_range_m=.6,
        policy='released_low_speed_rear_sweep_v1')
    if (raw_scan is None or not source_valid(scene) or
            any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x)
                for x in (speed,target,length,width,maximum_exterior_m)) or
            not 0<=speed<=.35 or not 0<target<=.35 or length<=0 or width<=0 or
            not 0<maximum_exterior_m<=.01):
        return result
    q=np.asarray(queries,dtype=float).reshape(-1,2);pad=np.asarray(padding,dtype=float)
    if (not len(q) or not np.all(np.isfinite(q)) or pad.shape!=(len(scene.rays),)
            or not np.all(np.isfinite(pad)) or np.any(pad<0)):
        return result
    exterior=np.abs(q[:,1])-width/2
    result.update(queries=len(q),exterior_max_m=float(exterior.max()))
    if (np.any(q[:,0]<-length/2) or np.any(q[:,0]>0) or np.any(exterior<=0)
            or np.any(exterior>maximum_exterior_m)):
        return {**result,'reason':'outside_released_rear_sweep'}
    answer=_bounded_nonreturn(scene,raw_scan,q,pad,result)
    if answer['accepted']:answer['reason']='bounded_released_rear_sweep_risk'
    elif answer['reason']=='wide_nonreturn' and float(exterior.max())<=.001:
        # 이미 쓰는 1 mm 후반부 기하와 전체 6도 예산의 조합이다. 공백의 수/합계,
        # 양옆 실제 반환·팽창·시각·원시 +inf는 같은 함수로 다시 검사한다.
        retry=_bounded_nonreturn(scene,raw_scan,q,pad,{**result,
            'maximum_gap_rad':math.radians(6),
            'maximum_exterior_m':min(.001,maximum_exterior_m),
            'policy':'released_low_speed_body_fringe_v2'})
        if retry['accepted']:
            retry.update(reason='bounded_released_body_fringe_risk',
                         previous_rejection='wide_nonreturn')
            return retry
    return answer
