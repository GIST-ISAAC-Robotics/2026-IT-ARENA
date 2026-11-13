"""횡단면 확인, 기존 정지/속도 우선, 기본 OFF와 명령 연결 조건."""
from dataclasses import replace
from types import SimpleNamespace as N
import pytest
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.solo_corner_approach import approach_speed,crossing_face
from test_solo_straight_budget import CFG

def rays(distance=2.,missing=(),jump=False,slope=0.):
    ys=[.10-i*.01 for i in range(21)]
    return [N(source_index=i+(4 if jump and i>=10 else 0),status='masked' if i in missing else 'return',hit_xy_m=None if i in missing else (distance+y*slope,y)) for i,y in enumerate(ys)]

def output(v,reason='local_follow'):return (v,.1,dict(reason=reason))

def test_current_crossing_only_and_lower_only():
    sc=N(rays=rays(),t_ref_s=1.,source_first_s=.88)
    cfg=replace(CFG,wall_offset_m=.505,solo_corner_approach_trial=True)
    v,detail=approach_speed(output(2.6),sc,cfg)
    assert 1.2<=v<2.6 and detail['distance_m']==pytest.approx(2.)
    for speed in (0.,.35,1.2):assert approach_speed(output(speed),sc,cfg)==(speed,None)
    assert approach_speed(output(2.,'follow_boundary_unconfirmed'),sc,cfg)==(2.,None)
    assert approach_speed(output(2.),N(rays=rays(),t_ref_s=1.,source_first_s=.5),cfg)==(2.,None)
    assert approach_speed(output(2.8),N(rays=rays(4.4),t_ref_s=1.,source_first_s=.88),cfg)[0]==2.8

@pytest.mark.parametrize('kind',['masked','jump','short','oblique','longitudinal'])
def test_unobserved_or_one_sided_face_is_not_a_crossing(kind):
    r=rays(missing=(10,) if kind=='masked' else (),jump=kind=='jump',slope=1 if kind=='oblique' else 0)
    if kind=='short':r=r[:10]
    if kind=='longitudinal':r=[N(source_index=i,status='return',hit_xy_m=(.5+i*.09,.04)) for i in range(21)]
    assert crossing_face(r,.15) is None

def test_default_and_explicit_config_contract():
    assert not FollowConfig().solo_corner_approach_trial
    with pytest.raises(ValueError):replace(FollowConfig(),solo_corner_approach_trial=True)
    with pytest.raises(ValueError):replace(CFG,solo_corner_approach_trial=1)
