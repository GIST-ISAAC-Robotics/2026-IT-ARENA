"""관측 검사 범위 바깥의 즉시 이웃만 사용한다. 추가 결측은 숨기지 않는다."""
from dataclasses import replace
import math
import numpy as np
import pytest
import importlib.util
from pathlib import Path

from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.partial_observation import PartialObservationPolicy
from test_local_follow import history


@pytest.fixture(autouse=True)
def explicit_candidate(monkeypatch):
    path=Path(__file__).with_name('partial_observation.candidate.py')
    spec=importlib.util.spec_from_file_location('front_edge_candidate',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(PartialObservationPolicy,'evaluate',module.PartialObservationPolicy.evaluate)


def native(first=.5, clockwise=False, side=1, neighbor=8., extend=False):
    angles=np.deg2rad(np.arange(-179.5,180.,.72))
    if clockwise:angles=angles[::-1]
    ranges=np.full(len(angles),8.)
    order=np.argsort(angles)
    front=order[np.abs(angles[order])<=math.radians(100)]
    edge=front[-1] if side>0 else front[0]
    adjacent=order[int(np.flatnonzero(order==edge)[0])+side]
    ranges[edge]=math.inf
    ranges[adjacent]=math.inf if extend else neighbor
    return MeasurementScan(tuple(ranges),tuple(angles),tuple(first+np.arange(len(angles))*.0002),
        time_basis='simulation_acquisition')


def warmed():
    p=PartialObservationPolicy(True,True,True,oblique_metric_enabled=True);h=history()
    initial=native(.4)
    initial=replace(initial,ranges_m=tuple(8. for _ in initial.ranges))
    assert p.evaluate(initial,[],h,.5,.3)['allowed']
    return p,h


@pytest.mark.parametrize('clockwise',[False,True])
@pytest.mark.parametrize('side',[-1,1])
def test_immediate_outer_return_bounds_edge_gap(clockwise,side):
    p,h=warmed();raw=native(clockwise=clockwise,side=side);before=raw.ranges
    r=p.evaluate(raw,[],h,.6,.3)
    assert r['allowed'] and r['reason']=='bounded_side_missing'
    assert r['speed_cap_mps']==.5 and not r['unknown_filled']
    assert r['residual_hidden_object_risk'] and raw.ranges==before


@pytest.mark.parametrize('neighbor',[math.inf,math.nan,-math.inf,.01,13.])
@pytest.mark.parametrize('side',[-1,1])
def test_invalid_outer_neighbor_does_not_bound_gap(neighbor,side):
    p,h=warmed();r=p.evaluate(native(side=side,neighbor=neighbor),[],h,.6,.3)
    assert not r['allowed'] and r['reason']=='unbounded_missing'


@pytest.mark.parametrize('side',[-1,1])
def test_gap_continuing_outside_front_is_not_cut_short(side):
    p,h=warmed();r=p.evaluate(native(side=side,extend=True),[],h,.6,.3)
    assert not r['allowed'] and r['reason']=='unbounded_missing'


def test_close_valid_outer_return_still_latches():
    p,h=warmed();r=p.evaluate(native(neighbor=.15),[],h,.6,.3)
    assert not r['allowed'] and r['reason']=='near_gap_boundary' and p.latched


def test_observation_history_still_required():
    p=PartialObservationPolicy(True,True,True,oblique_metric_enabled=True)
    r=p.evaluate(native(),[],history(),.6,.3)
    assert not r['allowed'] and r['reason']=='partial_history_warmup'


def test_expired_history_still_latches():
    p,h=warmed();r=p.evaluate(native(.9),[],h,1.,.3)
    assert not r['allowed'] and r['reason']=='partial_history_expired'


def test_near_obstacle_disappearing_at_edge_still_latches():
    p,h=warmed();raw=native();angle=next(a for r,a in zip(raw.ranges,raw.angles_rad) if math.isinf(r))
    p._remember([[.06+.16*math.cos(angle),.16*math.sin(angle)]],.5,.4,True)
    r=p.evaluate(raw,[],h,.6,.3)
    assert not r['allowed'] and r['reason']=='known_obstacle_latched'
