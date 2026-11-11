"""기존 위험 폭 안의 보수적 포괄 기하. 명령/기억/독립보호 승인은 별도다."""
from dataclasses import replace
import math
import sys
from pathlib import Path

import pytest

ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from probe import envelope


def fixture(reverse=False):
    angles=[-math.pi+i*math.tau/500 for i in range(500)]
    if reverse:angles=angles[::-1]
    ranges=[8.]*500
    for target in (25.,28.):
        index=min(range(500),key=lambda i:abs(math.degrees(angles[i])-target))
        ranges[index]=math.inf
    raw=MeasurementScan(tuple(ranges),tuple(angles),tuple(.5+i*.0002 for i in range(500)),
        time_basis='simulation_acquisition')
    history=MotionHistory(MotionConfig())
    for series in (history.wheels,history.gyro):
        series.times=[i*.01 for i in range(70)];series.values=[0.]*70
    return raw,history


def check(raw,history,**extra):
    return envelope(raw,history,extra.pop('now',.62),extra.pop('speed',.2),
        extra.pop('steering',.25),extra.pop('previous',.25),**extra)


@pytest.mark.parametrize('reverse',[False,True])
def test_enclosing_wedge_keeps_real_middle_points_and_raw_arrays(reverse):
    raw,h=fixture(reverse);before=(raw.ranges_m,raw.angles_rad,raw.acquired_at_s)
    result=check(raw,h)
    assert result['eligible'] and result['runs']==2 and result['observed_internal_points']>0
    assert result['hidden_width_m']<=.05 and result['source_span_deg']>3.
    assert not result['unknown_filled'] and result['scope']=='geometry_only_not_command_permission'
    assert before==(raw.ranges_m,raw.angles_rad,raw.acquired_at_s)


@pytest.mark.parametrize('value',[float('nan'),-math.inf,.01])
def test_invalid_or_near_returns_are_not_promoted_to_missing(value):
    raw,h=fixture();v=list(raw.ranges);i=next(i for i,x in enumerate(v) if math.isinf(x));v[i]=value
    assert not check(replace(raw,ranges_m=tuple(v)),h)['eligible']


def test_middle_obstacle_is_preserved_as_a_rejection():
    raw,h=fixture();v=list(raw.ranges);ids=[i for i,x in enumerate(v) if math.isinf(x)]
    v[(ids[0]+ids[1])//2]=.3
    result=check(replace(raw,ranges_m=tuple(v)),h)
    assert not result['eligible'] and result['reason']=='near_observed_point'


@pytest.mark.parametrize('case',['fast','stale','separated','one_run','no_missing'])
def test_existing_bounds_are_retained(case):
    raw,h=fixture();kwargs={};v=list(raw.ranges)
    if case=='fast':kwargs['speed']=.61
    elif case=='stale':kwargs['now']=.8
    elif case=='separated':v[min(range(500),key=lambda i:abs(math.degrees(raw.angles_rad[i])+25))]=math.inf
    elif case=='one_run':v[v.index(math.inf)]=8.
    elif case=='no_missing':v=[8.]*500
    assert not check(replace(raw,ranges_m=tuple(v)),h,**kwargs)['eligible']


def test_rotation_and_translation_origins_are_accounted_for():
    raw,h=fixture();h.gyro.values=[4.]*len(h.gyro.times);h.wheels.values=[.4]*len(h.wheels.times)
    result=check(raw,h,speed=.4)
    assert result['origin_span_m']>0
    assert result['hidden_width_m']==pytest.approx(result['angular_chord_m']+result['origin_span_m'])


def test_expanded_body_can_exceed_same_physical_width():
    raw,h=fixture()
    result=check(raw,h,length=.9,width=.6)
    assert not result['eligible'] and result['reason']=='physical_width_exceeded'
