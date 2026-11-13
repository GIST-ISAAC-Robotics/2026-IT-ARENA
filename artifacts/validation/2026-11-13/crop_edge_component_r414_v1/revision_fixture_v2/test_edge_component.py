"""실제 Recovery+structure 상속 구성의 고정 반례. 실행은 지정 담당자만 수행한다."""
import copy
from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

A=Path(__file__).resolve().parent.parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import arena_autonomy
arena_autonomy.__path__.insert(0,str(A/'staged/arena_autonomy'))
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from arena_autonomy.known_structure import structure_policy
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from arena_autonomy.scan_input import project_scan
from arena_autonomy.recovery_geometry import timed_missing_sweep
from arena_autonomy import numeric_kernels
from arena_autonomy.follow_trace import freeze,comparable_decision

HEADER=json.loads((A/'fixture_header.json').read_bytes())
CFG=HEADER['follow_config']
BODY=dict(wheelbase=CFG['wheelbase_m'],length=CFG['length_m'],width=CFG['width_m'])


@pytest.fixture(autouse=True)
def python_kernel():
    numeric_kernels.configure('python')
    yield
    numeric_kernels.configure('python')


def policy(flag=True, sectors=None):
    cls=structure_policy(RecoveryPolicy,sectors or CFG['known_structure_sectors_json'])
    return cls(True,True,True,True,crop_edge_component_trial=flag)


def history(policy_name='verified_only'):
    h=MotionHistory(MotionConfig(**HEADER['motion_config']),scan_time_policy=policy_name)
    for series in (h.wheels,h.gyro):
        series.times=[i*.005 for i in range(201)];series.values=[0.]*201
    return h


def measurements(first=0., *, missing=False, close=(), close_range=.35, far_near=False,
                 end=104., dense=False, cw=False, seam=False, outside_bad=None, extra=False):
    degrees=[-179.8+i*.72 for i in range(500)]
    for index,value in zip(range(388,395),(99.4,100.2,101.,101.7,102.4,103.2,104.)):
        degrees[index]=value
    if end>104.:
        degrees=[-180+i*.6 for i in range(600)]
    if dense:
        degrees=sorted([x for x in degrees if not 99.4<=x<=104.]+[99.4+i*.1 for i in range(47)])
    if cw:degrees=list(reversed(degrees))
    if seam:
        split=min(range(len(degrees)),key=lambda i:abs(degrees[i]-102.4))
        degrees=degrees[split:]+degrees[:split]
    values=[.57]*len(degrees)
    for i,angle in enumerate(degrees):
        if any(abs(angle-wanted)<1e-7 for wanted in close):values[i]=close_range
        if missing and 99.4-1e-7<=angle<=end+1e-7:values[i]=math.inf
        if extra and 80.-1e-7<=angle<=82.+1e-7:values[i]=math.inf
        if outside_bad is not None and abs(angle-102.4)<1e-7:values[i]=outside_bad
    if far_near:
        indices=sorted(range(len(degrees)),key=lambda i:degrees[i])
        flank=next(i for i in indices if degrees[i]>end+1e-7);values[flank]=.1
    scan=MeasurementScan(tuple(values),tuple(math.radians(x) for x in degrees),
        tuple(first+i*.08/(len(values)-1) for i in range(len(values))),time_basis='simulation_acquisition')
    return scan


def evaluate(p,h,scan,now,speed=0.,steering=0.,previous=0.):
    before=freeze(scan.ranges)
    points=project_scan(scan,h,now,timing_verified=h.scan_time_policy=='verified_only')[0]
    result=p.evaluate(scan,points,h,now,speed,steering,previous,**BODY)
    assert freeze(scan.ranges)==before
    return result


def warm(*,close=(),distance=.35,flag=True,sectors=None,**kwargs):
    p=policy(flag,sectors);h=history()
    old=measurements(close=close,close_range=distance,**kwargs)
    assert evaluate(p,h,old,.1)['allowed']
    return p,h,old


@pytest.mark.parametrize('kernel',['python','native_v1'])
@pytest.mark.parametrize('speed,steering',[(0.,0.),(.5,.25)])
def test_f1_outside_three_points_are_captured_before_remember(kernel,speed,steering):
    numeric_kernels.configure(kernel)
    p,h,old=warm(close=(101.,101.7,102.4))
    raw=measurements(.1,missing=True)
    result=evaluate(p,h,raw,.2,speed,steering)
    assert not result['allowed'] and p.latched
    assert p.latch_reason=='previous_point_in_missing_sweep'
    assert p.anchor_threat_summary['unique_points']==3 and p.anchor_threat_summary['complete']
    assert len(p.anchor_probe.retained)==3
    # 최초 위협의 원천과 재관측 후 rolling anchor의 현재 시각은 다른 계약이다.
    assert p.anchor_threat_summary['reference_s']==.1
    assert p.anchor_threat_summary['source_first_s']==0.
    assert p.anchor_probe.created==.1
    assert p.anchor_probe.anchor[1:]==(.2,.1)
    proof=result['crop_edge_component']
    assert len(proof['source_indices'])==7 and len(proof['inside_source_indices'])==1
    assert result['timed_missing_diagnostic']['missing_rays']==7
    assert p.previous[1:]==(.2,.1)  # 현재점 갱신 뒤에도 옛 전체 위협은 별도 기존 anchor에 남는다.
    assert not result['unknown_filled']


def direct_threat(scan,old,index,radius,now=.2):
    angle=scan.angles_rad[index];cfg=HEADER['motion_config']
    point=[[cfg['lidar_x_m']+radius*math.cos(angle),cfg['lidar_y_m']+radius*math.sin(angle)]]
    result=timed_missing_sweep(point,[old.acquired_at_s[index]],
        [[cfg['lidar_x_m'],cfg['lidar_y_m']]],[angle],[scan.acquired_at_s[index]],now,
        math.radians(.6)/2,.6,0.,now-scan.acquired_at_s[0],0.,0.,**BODY)
    return result['possible'][0]


def test_f2_direct_ray_outer_margin_is_retained_without_margin_tuning():
    numeric_kernels.configure('native_v1')
    old=measurements(end=105.);raw=measurements(.1,missing=True,end=105.)
    index=min(range(len(raw.ranges)),key=lambda i:abs(math.degrees(raw.angles_rad[i])-105.))
    low,high=.2,.8
    assert direct_threat(raw,old,index,low) and not direct_threat(raw,old,index,high)
    for _ in range(24):
        mid=(low+high)/2
        if direct_threat(raw,old,index,mid):low=mid
        else:high=mid
    distance=low-.01
    assert direct_threat(raw,old,index,distance)
    values=list(old.ranges);values[index]=distance;old=replace(old,ranges_m=tuple(values))
    p=policy();h=history();assert evaluate(p,h,old,.1)['allowed']
    result=evaluate(p,h,raw,.2)
    assert not result['allowed'] and p.latch_reason=='previous_point_in_missing_sweep'
    assert p.anchor_threat_summary['unique_points']==1
    assert index in result['crop_edge_component']['combined_unknown_source_indices']
    print(json.dumps(dict(fixture='F2',direct_threshold_interval_m=[low,high],point_radius_m=distance)))


@pytest.mark.parametrize('cw',[False,True])
def test_supported_component_is_bounded_risk_not_filled(cw):
    p,h,_=warm(cw=cw);raw=measurements(.1,missing=True,cw=cw)
    result=evaluate(p,h,raw,.2)
    assert result['allowed'] and not p.latched and result['speed_cap_mps']<=.5
    assert not result['unknown_filled'] and result['residual_hidden_object_risk']
    assert result['crop_edge_component']['full_width_deg']<=6.+1e-6


def test_incrop_hidden_point_still_latches():
    p,h,_=warm(close=(99.4,));r=evaluate(p,h,measurements(.1,missing=True),.2)
    assert not r['allowed'] and p.latch_reason=='previous_point_in_missing_sweep'


def test_real_point_four_second_gap_still_expires():
    p,h,_=warm();r=evaluate(p,h,measurements(.4,missing=True),.5)
    assert not r['allowed'] and p.latch_reason=='partial_history_expired'


@pytest.mark.parametrize('change',[
    dict(end=106.2),dict(extra=True),dict(dense=True),dict(seam=True),
    dict(outside_bad=.01),dict(outside_bad=13.)])
def test_bad_component_or_combined_budget_keeps_original_denial(change):
    p,h,_=warm(**{k:v for k,v in change.items() if k in ('end','dense','seam')})
    before=freeze((p.previous,p.previous_distinct,p.last_source))
    r=evaluate(p,h,measurements(.1,missing=True,**change),.2)
    assert not r['allowed'] and r['reason']=='unbounded_missing' and 'crop_edge_component' not in r
    assert freeze((p.previous,p.previous_distinct,p.last_source))==before


def test_structure_ray_cannot_be_crossed_even_if_not_excused():
    sectors=json.loads(CFG['known_structure_sectors_json'])['sectors_deg']+[[102.4,.1]]
    p,h,_=warm(sectors=json.dumps(sectors));r=evaluate(p,h,measurements(.1,missing=True),.2)
    assert not r['allowed'] and r['reason']=='unbounded_missing'


def test_far_flank_near_return_keeps_existing_latch():
    p,h,_=warm();r=evaluate(p,h,measurements(.1,missing=True,far_near=True),.2)
    assert not r['allowed'] and p.latch_reason=='near_gap_boundary'


@pytest.mark.parametrize('basis,clock',[('unverified_estimate','host_estimated_v1'),
                                      ('validated_sensor_acquisition','verified_only')])
def test_non_simulation_time_does_not_activate_component(basis,clock):
    p,h,_=warm();h.scan_time_policy=clock
    raw=replace(measurements(.1,missing=True),time_basis=basis)
    r=evaluate(p,h,raw,.2)
    assert not r['allowed'] and 'crop_edge_component' not in r


def test_array_and_incomplete_native_metadata_cannot_activate_component():
    p,h,_=warm();raw=measurements(.1,missing=True)
    legacy=NS(ranges=list(raw.ranges),range_min=raw.range_min,range_max=raw.range_max,
        angle_min=math.radians(-179.8),angle_increment=math.radians(.72),time_increment=.08/499,
        header=NS(frame_id='laser_frame',stamp=NS(sec=0,nanosec=100000000)))
    r=evaluate(p,h,legacy,.2)
    assert r['reason']=='unbounded_missing' and 'crop_edge_component' not in r
    projected=project_scan(raw,h,.2,timing_verified=True)[0]
    angles=list(raw.angles_rad);angles[391]+=math.radians(3.)
    bad=replace(raw,angles_rad=tuple(angles))
    before=freeze((p.previous,p.last_source))
    r=p.evaluate(bad,projected,h,.2,0.,**BODY)
    assert not r['allowed'] and 'crop_edge_component' not in r
    assert freeze((p.previous,p.last_source))==before


@pytest.mark.parametrize('lower,upper',[(94.,104.),(0.,1.4)])
def test_original_metric_and_central_paths_are_unchanged(lower,upper):
    off,h,_=warm(flag=False);on,_,_=warm()
    raw=measurements(.1)
    raw=replace(raw,ranges_m=tuple(math.inf if lower<=math.degrees(a)<=upper else r
                                  for a,r in zip(raw.angles_rad,raw.ranges)))
    assert evaluate(off,h,raw,.2)==evaluate(on,h,raw,.2)


def test_same_source_extra_age_waits_without_permanent_latch():
    p,h,_=warm(close=(101.7,),distance=.5)
    raw=measurements(.1,missing=True)
    assert evaluate(p,h,raw,.2)['allowed'] and not p.latched
    before=freeze((p.previous,p.previous_distinct,p.last_source))
    r=evaluate(p,h,raw,.28)
    assert not r['allowed'] and r['reason']=='same_scan_missing_wait'
    assert not p.latched and p.anchor_probe.anchor is None
    assert freeze((p.previous,p.previous_distinct,p.last_source))==before


def test_off_base_instance_reports_and_memory_match_original():
    spec=importlib.util.spec_from_file_location('_original_partial',ROOT/'src/arena_autonomy/arena_autonomy/partial_observation.py')
    original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
    before=original.PartialObservationPolicy(True,True,True,True)
    after=PartialObservationPolicy(True,True,True,True,crop_edge_component_trial=False)
    h=history()
    for raw,now in [(measurements(),.1),(measurements(.1,missing=True),.2)]:
        assert evaluate(before,h,raw,now)==evaluate(after,h,raw,now)
        assert freeze(vars(before))==freeze(vars(after))
    with pytest.raises(ValueError):PartialObservationPolicy(crop_edge_component_trial=1)
