"""후방 현재 면의 긴 관측 창이 전방 장애물/미관측/시간 고장을 대신하지 않는지 검사한다."""
from dataclasses import replace
from types import SimpleNamespace as NS
from unittest.mock import patch
import math
import numpy as np
import pytest
from arena_autonomy.bypass_corridor_trial import CorridorPolicy
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.rear_face_motion import RearFaceMotion


def frame(t,inward=0.,epoch=0):
    cfg=replace(FollowConfig(),wall_offset_m=.505)
    first=round(t-.2,8);sec=math.floor(first);nano=round((first-sec)*1e9)
    # 연속 후방 면과 독립 전방 장애물. source id가 섞여도 앞쪽은 선택하지 않는다.
    points=[(-.39+i*.01,-.22+inward) for i in range(29)]+[(.15,0.)]
    rays=[NS(source_index=i,hit_xy_m=p,status='return',acquired_at_s=first+.02) for i,p in enumerate(points)]
    scene=NS(rays=rays,t_ref_s=t,source_first_s=first,source_epoch=epoch)
    scan=NS(header=NS(stamp=NS(sec=sec,nanosec=nano)),ranges=[1.]*len(points))
    wall=dict(reason='current_connected_straight_wall_fit',slope=0.,intercept_m=.285,observed_x_m=[-.05,1.5])
    proof=dict(source_ray_indices=list(range(len(points))),proof={'short_piece_static':False})
    original=np.full(len(points),.055+.5*.18)
    return cfg,scene,scan,wall,proof,original


def apply(observer,t,*,active=True,missing=False,stale=False,inward=0.,epoch=0):
    cfg,scene,scan,wall,proof,original=frame(t,inward,epoch)
    if stale:scan.header.stamp.nanosec+=1_000_000
    with patch('arena_autonomy.rear_face_motion.classify',return_value=(None,None,[])), \
         patch('arena_autonomy.rear_face_motion.observed_rear_wall_face',return_value=None if missing else proof), \
         patch('arena_autonomy.rear_face_motion.transport_points',side_effect=lambda x,*a:x):
        result,detail=observer.apply(scene,cfg,'right',wall,scan,(1,),object(),original,active=active)
    assert np.all(result>=.055) and np.all(result<=original)
    assert result[-1]==original[-1]  # 후방 안정 근거로 현재 전방 점의 여유를 줄이지 않는다.
    np.testing.assert_allclose(original,np.full(len(original),.145),rtol=0,atol=1e-15)
    return result,detail


def test_reuses_do_not_accumulate_evidence_and_front_obstacle_padding_is_unchanged():
    observer=RearFaceMotion(CorridorPolicy())
    for k in range(8):
        for _ in range(3):
            _,detail=apply(observer,round(.5+.1*k,8))
            assert not detail['stable'] and detail['changed']==0
    result,detail=apply(observer,1.3)
    assert detail['stable'] and detail['changed']==29 and max(result[:-1])<.13
    assert not detail['unknown_filled'] and not detail['static_identity_claimed']


@pytest.mark.parametrize('lost',['missing','stale','inactive','epoch','gap','inward'])
def test_current_support_or_time_loss_drops_the_reduction(lost):
    observer=RearFaceMotion(CorridorPolicy())
    for k in range(9):apply(observer,round(.5+.1*k,8))
    kwargs={}
    if lost=='missing':kwargs['missing']=True
    elif lost=='stale':kwargs['stale']=True
    elif lost=='inactive':kwargs['active']=False
    elif lost=='epoch':kwargs['epoch']=1
    elif lost=='inward':kwargs['inward']=.04
    _,detail=apply(observer,1.8 if lost=='gap' else 1.4,**kwargs)
    assert not detail['stable'] and detail['changed']==0
