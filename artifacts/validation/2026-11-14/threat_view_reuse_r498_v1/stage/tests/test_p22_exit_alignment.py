import math
from unittest.mock import patch
import numpy as np

from arena_autonomy.p22_exit_alignment import propose
from arena_autonomy.p22_route_trial import body_path


def lines():
    def line(x,y,angle,span):
        v=np.array([math.cos(angle),math.sin(angle)])
        return dict(m=np.array([x,y]),v=v,span=span,lo=-span/2,hi=span/2,heading_deg=math.degrees(angle))
    return [line(.35,-.25,0.,.7),line(.35,.35,0.,.7),line(1.72,0.,math.pi/2,2.)]


def test_preserves_start_and_smoothly_aligns_to_observed_center():
    path=np.column_stack((np.linspace(-.3,.65,96),np.zeros(96)))
    with patch('arena_autonomy.p22_exit_alignment.segments',return_value=lines()):
        rows=propose(None,0.,path)
    assert len(rows)==1
    q=rows[0];body=body_path(q,.135)
    assert np.allclose(body[:31],path[:31],atol=1e-12)
    assert abs(body[len(path)-1,1]-.05)<1e-12
    poses=np.asarray(q['poses']);future=poses[30:]
    assert np.max(abs(future[:,3]))<=.37
    assert np.all(abs(np.diff(future[:,3]))*.65<=3*np.diff(future[:,4])+1e-8)
    assert abs(q['retained_offset_m']-.05)<1e-12


def test_too_short_or_wrong_direction_is_not_corrected_by_force():
    short=np.column_stack((np.linspace(-.1,.2,31),np.zeros(31)))
    wrong=np.column_stack((np.linspace(-.3,.65,96),np.linspace(-.3,.65,96)*math.tan(math.radians(10))))
    with patch('arena_autonomy.p22_exit_alignment.segments',return_value=lines()):
        assert propose(None,0.,short)==[]
        assert propose(None,0.,wrong)==[]


def test_absent_or_ambiguous_opposite_wall_does_not_create_free_space():
    path=np.column_stack((np.linspace(-.3,.65,96),np.zeros(96)))
    with patch('arena_autonomy.p22_exit_alignment.segments',return_value=lines()[::2]):
        assert propose(None,0.,path)==[]
    with patch('arena_autonomy.p22_exit_alignment.segments',return_value=lines()+[lines()[1]]):
        assert propose(None,0.,path)==[]
