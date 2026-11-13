"""실제 부분 관측 함수의 제한 반례. 실행은 지정 담당자만 수행한다."""
import copy
from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.partial_observation import PartialObservationPolicy as Original
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory

spec=importlib.util.spec_from_file_location('_r410_partial',A/'staged/arena_autonomy/partial_observation.py')
candidate=importlib.util.module_from_spec(spec);spec.loader.exec_module(candidate)
Candidate=candidate.PartialObservationPolicy


def history():
    result=MotionHistory(MotionConfig(lidar_x_m=0.,lidar_y_m=0.))
    for series in (result.wheels,result.gyro):
        series.times=[i*.01 for i in range(101)];series.values=[0.]*101
    return result


def scan(kind='array',first=.1,*,missing=True,neighbor=2.,outside_missing=False):
    angles=[math.radians(-180+i*.72) for i in range(501)]
    if kind=='nonuniform':angles=[a+.001*math.sin(i) for i,a in enumerate(angles)]
    if kind=='cw':angles=list(reversed(angles))
    order=sorted(range(len(angles)),key=lambda i:(angles[i]+math.pi)%math.tau-math.pi)
    front=[i for i in order if abs((angles[i]+math.pi)%math.tau-math.pi)<=math.radians(100)+1e-6]
    edge=front[-1];at=order.index(edge);flanks=[order[at-1],order[at+1]]
    ranges=[2.]*len(angles)
    if missing:ranges[edge]=float('inf')
    ranges[flanks[1]]=float('inf') if outside_missing else neighbor
    if kind=='array':
        result=NS(ranges=ranges,range_min=.05,range_max=12.,angle_min=angles[0],angle_increment=math.radians(.72),
            time_increment=.08/(len(ranges)-1),header=NS(frame_id='laser_frame',stamp=NS(sec=0,nanosec=round(first*1e9))))
    else:
        result=MeasurementScan(tuple(ranges),tuple(angles),tuple(first+i*.08/(len(ranges)-1) for i in range(len(ranges))),
            time_basis='simulation_acquisition')
    return result,angles,edge,flanks


def points(value,angles):
    return np.array([(r*math.cos(a),r*math.sin(a)) for r,a in zip(value.ranges,angles)
                     if math.isfinite(r) and value.range_min<=r<=value.range_max])


def evaluate(cls,kind='array',*,neighbor=2.,outside_missing=False,warmup=True,latched=False,old_threat=False,expired=False):
    policy=cls(allow_narrow_forward=True,lateral_metric_enabled=True)
    hist=history()
    if warmup:
        old,angles,edge,_=scan(kind,first=0.,missing=False)
        if old_threat:
            ranges=list(old.ranges);ranges[edge]=.3
            if isinstance(old,MeasurementScan):old=replace(old,ranges_m=tuple(ranges))
            else:old.ranges=ranges
        old_points=points(old,angles)
        assert policy.evaluate(old,old_points,hist,.1,.2)['allowed']
    if latched:policy._latch('existing_threat',.1)
    value,angles,_,_=scan(kind,first=.4 if expired else .1,neighbor=neighbor,outside_missing=outside_missing)
    before=copy.deepcopy(value.ranges)
    result=policy.evaluate(value,points(value,angles),hist,.5 if expired else .2,.2)
    assert all((math.isnan(a) and math.isnan(b)) or a==b for a,b in zip(before,value.ranges))
    return result,policy


@pytest.mark.parametrize('kind',['array','ccw','cw','nonuniform'])
def test_only_lost_finite_flank_reaches_existing_history_decision(kind):
    before,_=evaluate(Original,kind);after,_=evaluate(Candidate,kind)
    assert before['reason']=='unbounded_missing' and not before['allowed']
    assert after['allowed'] and after['reason']=='bounded_side_missing'
    for key in ('unknown_samples','max_gap_deg','total_gap_deg','speed_cap_mps','history_distance_m','near_boundary_range_m'):
        assert after[key]==before[key]
    assert after['unknown_filled'] is False


@pytest.mark.parametrize('kind',['array','ccw','cw','nonuniform'])
def test_unknown_continuing_outside_crop_remains_exact_rejection(kind):
    before,bp=evaluate(Original,kind,outside_missing=True);after,ap=evaluate(Candidate,kind,outside_missing=True)
    assert before==after and after['reason']=='unbounded_missing' and not after['allowed']
    assert bp.latched==ap.latched and bp.last_source==ap.last_source


@pytest.mark.parametrize('neighbor',[float('nan'),float('inf'),float('-inf'),.01,13.])
def test_nonreturn_or_invalid_flank_never_becomes_return(neighbor):
    before,_=evaluate(Original,neighbor=neighbor);after,_=evaluate(Candidate,neighbor=neighbor)
    assert before==after and not after['allowed']


@pytest.mark.parametrize('kwargs,reason',[
    ({'warmup':False},'partial_history_warmup'),({'expired':True},'partial_history_expired'),
    ({'latched':True},'known_obstacle_latched'),({'old_threat':True},'known_obstacle_latched'),
    ({'neighbor':.1},'near_gap_boundary')])
def test_existing_history_threat_and_near_distance_gates_still_apply(kwargs,reason):
    after,_=evaluate(Candidate,**kwargs)
    assert not after['allowed'] and after['reason']==reason


@pytest.mark.parametrize('order,front,run,valid,expected',[
    ([0,1,2,3,4,5],[1,2,3,4],[3],[True,True,True,True,False,True],(3,5)),
    ([5,4,3,2,1,0],[4,3,2,1],[0],[True,True,True,True,False,True],(5,3)),
    ([4,5,0,1,2,3],[0,1,2],[0],[False,True,True,True,True,True],None),
    ([0,2,1,3,4,5],[1,3,4],[0],[True,False,True,True,True,True],None),
    ([0,1,2,3],[0,1,2],[0],[False,True,True,True],None),
    ([0,1,2,3,4,5],[1,2,3],[2],[True,True,True,False,False,True],None),
])
def test_raw_adjacency_not_sorting_or_seam_synthetic_neighbors(order,front,run,valid,expected):
    frozen=copy.deepcopy((order,front,run,valid))
    assert candidate._cropped_gap_flanks(order,front,run,valid)==expected
    assert (order,front,run,valid)==frozen


def test_fully_observed_non_target_report_and_memory_are_unchanged():
    value,angles,_,_=scan('cw',missing=False);hist=history();cloud=points(value,angles)
    bp=Original();ap=Candidate()
    assert bp.evaluate(value,cloud,hist,.2,.2)==ap.evaluate(value,cloud,hist,.2,.2)
    assert bp.last_source==ap.last_source and bp.latched==ap.latched
    np.testing.assert_array_equal(bp.previous[0],ap.previous[0])
