"""State/geometry contracts for the isolated candidate; not a sensor or driving proof."""
from dataclasses import replace
import numpy as np
import pytest
import arena_autonomy.local_follow as lf
from test_traffic_boundary_refresh import CONFIG
from test_local_follow import scene,history


def ctl(on=True):return lf.LocalFollow(lf.FollowConfig(**CONFIG,piecewise_wall_trial=True,traffic_boundary_refresh_trial=on))


def base(c,s,side='left'):return c._baseline_command(s,history(),.35,side)

def current(t,**kw):
    s=scene(t,**kw)
    return replace(s,rays=tuple(r for r in s.rays if not (r.hit_xy_m[1]>.1 and r.hit_xy_m[0]>.15)))


def retained_right(c):
    for t in (.4,.5,.6,.7,.8):result=base(c,scene(t),'right')
    assert result[0]>0 and c.path_wall=='right'
    # Unit-state injection: previous bilateral selection already retained the right boundary.
    c.side='left'


def hide_preferred(monkeypatch,short_other=False):
    original=lf.local_path
    def observed(points,side,*args,**kwargs):
        if side=='left':return None,{'reason':'wall_segment_short'}
        path,info=original(points,side,*args,**kwargs)
        if short_other:info={**info,'boundary_observed_ahead_m':.4}
        return path,info
    monkeypatch.setattr(lf,'local_path',observed)


def test_only_long_current_previously_selected_wall_refreshes(monkeypatch):
    off,on=ctl(False),ctl()
    retained_right(off);retained_right(on);hide_preferred(monkeypatch)
    for c in (off,on):
        for t in (.9,1.,1.1):r=base(c,current(t))
        if c is off:assert r[0]==0
        else:
            assert r[0]>0 and r[2]['boundary_selection']=='current_retained_wall_refresh'
            assert c.path_source==pytest.approx(1.) and c.path_wall=='right'


@pytest.mark.parametrize('cause',['missing','expired','disagrees','different_wall','epoch','side'])
def test_retained_refresh_requires_actual_continuity(monkeypatch,cause):
    c=ctl()
    if cause!='missing':retained_right(c)
    if cause=='expired':c.path_source=.5
    if cause=='disagrees':c.path[:,1]+=.16
    if cause=='different_wall':c.path_wall='left'
    hide_preferred(monkeypatch)
    for t in (.9,1.,1.1):
        s=current(t,epoch=1 if cause=='epoch' else 0)
        r=base(c,s,'right' if cause=='side' else 'left')
        assert r[2].get('boundary_selection')!='current_retained_wall_refresh'
    assert r[0]==0


def test_short_other_wall_cannot_extend_recent_path(monkeypatch):
    c=ctl();retained_right(c);hide_preferred(monkeypatch,True)
    for t in (.9,1.,1.1):r=base(c,current(t))
    assert r[0]==0


def test_current_path_update_does_not_override_occupied_or_missing_rays(monkeypatch):
    c=ctl();retained_right(c);hide_preferred(monkeypatch)
    r=base(c,current(.9,obstacle_x=.1))
    assert r[0]==0 and r[2]['reason']=='follow_gap_hold'
    s=current(1.)
    unknown=replace(s,rays=tuple(replace(ray,status='unknown',hit_xy_m=None) for ray in s.rays))
    r=base(c,unknown)
    assert r[0]==0 and r[2]['reason']=='follow_front_unobserved'
