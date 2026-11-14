"""r538 후보 읽기 전용 독립 반례. 기억 존재 자체의 전역0.1 전제는 적용하지 않는다."""
import copy,importlib.util,json,math
from pathlib import Path
import numpy as np
import pytest
from arena_autonomy.guard_memory_command import limited_memory_command
from arena_autonomy.scan_input import project_scan
from test_guard_memory_command import seeded,PARTIAL,guard_node
import test_independent_guard_route as fixture

A=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('r538_frozen_before',A.parent/'before.py')
BASE=importlib.util.module_from_spec(spec);spec.loader.exec_module(BASE)
ROWS=[]


def scan_with_unobserved(points,first=.5):
    raw=fixture.scan(first,ranges=[2.]*500)
    for x,y in points:
        angle=math.atan2(y,x-.06)
        index=round((angle+math.pi)/math.tau*500)%500
        for offset in (-1,0,1):raw.ranges[(index+offset)%500]=float('nan')
    return raw


def evaluate(p,h,raw,*,requested=1.,measured=0.,steering=0.,previous=0.,now=.6,partial=None,baseline=False):
    cloud=project_scan(raw,h,now,timing_verified=True)[0]
    fn=BASE.limited_memory_command if baseline else limited_memory_command
    return fn(p,PARTIAL if partial is None else partial,raw,cloud,h,now,requested,measured,
        steering,previous,.128,.207,.15)


def record(name,result,p):
    ROWS.append(dict(case=name,result=result,active_points=p.guard_memory.points.tolist(),
        original_points=p.guard_memory.original.tolist()))


@pytest.mark.parametrize('point',[(-.40,0.),(.06,.38),(.06,-.38)])
def test_rear_or_side_unobserved_memory_preserved_with_normal_cap(point):
    p,h=seeded((point,));raw=scan_with_unobserved((point,));before=np.array(raw.ranges)
    evidence=copy.deepcopy(p.latch_evidence);result=evaluate(p,h,raw)
    assert result['accepted'] and result['speed_cap_mps']==1.
    assert not result['memory_relevant_to_command'] and result['normal_cap_restored']
    assert not result['all_retained_points_currently_resolved'] and result['retained_points']==1
    assert p.latched and p.latch_evidence==evidence
    np.testing.assert_array_equal(p.guard_memory.points,[point]);np.testing.assert_array_equal(raw.ranges,before)
    record('unobserved_outside_'+str(point),result,p)


@pytest.mark.parametrize('measured',[0.,.5])
def test_forward_point_35cm_crawls_or_stops_using_measured_braking(measured):
    p,h=seeded(((.35,0.),));raw=scan_with_unobserved(((.35,0.),))
    result=evaluate(p,h,raw,measured=measured)
    assert result['memory_relevant_to_command'] and result['retained_points']==1
    if measured==0.:assert result['accepted'] and result['speed_cap_mps']==.1
    else:assert not result['accepted'] and result['braking_speed_mps']<measured
    record('forward35cm_measured_'+str(measured),result,p)


def test_previous_steering_alone_retains_imminent_stop():
    p,h=seeded(((.13,.13),));raw=scan_with_unobserved(((.13,.13),))
    result=evaluate(p,h,raw,previous=.45)
    assert result['memory_relevant_to_command'] and not result['accepted']
    assert result['current_memory_clearance_m']>result['previous_memory_clearance_m']
    assert result['previous_memory_clearance_m']<=.06
    record('previous_steering_only',result,p)


def test_one_forward_point_prevents_normal_cap_with_unrelated_side_point():
    points=((.06,-.38),(.35,0.));p,h=seeded(points);raw=scan_with_unobserved(points)
    result=evaluate(p,h,raw)
    assert result['accepted'] and result['speed_cap_mps']==.1 and result['retained_points']==2
    assert result['memory_relevant_to_command'] and not result['normal_cap_restored']
    record('mixed_side_and_forward',result,p)


def test_current_raw_near_obstacle_stops_even_when_memory_irrelevant():
    points=((.06,-.38),);p,h=seeded(points);raw=scan_with_unobserved(points);raw.ranges[250]=.05
    result=evaluate(p,h,raw)
    assert not result['memory_relevant_to_command'] and not result['accepted']
    assert result['current_raw_clearance_m']==0.
    record('raw_imminent_with_irrelevant_memory',result,p)


@pytest.mark.parametrize('condition',['STOP','structure_cap_zero','partial_cap_zero'])
def test_existing_stop_and_cap_zero_are_not_overridden(condition):
    p,h=seeded(((.06,-.38),));raw=scan_with_unobserved(((.06,-.38),))
    options={'requested':0.} if condition=='STOP' else {'partial':{
        **PARTIAL,**({'known_structure':{'speed_cap_mps':0.}} if condition=='structure_cap_zero'
                    else {'speed_cap_mps':0.})}}
    result=evaluate(p,h,raw,**options)
    assert not result['accepted'] and result['speed_cap_mps']==0. and p.latched
    record(condition,result,p)


def test_same_retained_side_point_becomes_relevant_after_steering_changes():
    point=(.06,-.38);p,h=seeded((point,));raw=scan_with_unobserved((point,))
    first=evaluate(p,h,raw);assert first['speed_cap_mps']==1.
    second=evaluate(p,h,raw,now=.62,steering=-.55,previous=-.55,measured=.1)
    assert second['current_ray_clearance']['reason']=='same_scan_no_new_ray_evidence'
    assert second['memory_relevant_to_command'] and second['accepted'] and second['speed_cap_mps']==.1
    assert len(p.guard_memory.points)==1 and p.guard_memory.reference==.62 and p.latched
    np.testing.assert_array_equal(p.guard_memory.original,[point])
    record('same_memory_turns_into_path',second,p)


@pytest.mark.parametrize('requested',[.5,.8,1.5])
def test_requested_speed_horizon_does_not_create_new_zero_for_low_final_target(requested):
    point=(.35,0.);raw=scan_with_unobserved((point,));pair=[]
    for baseline in (True,False):
        p,h=seeded((point,))
        if baseline:p.guard_memory=BASE.RetainedCommandMemory([point],.4,.3,('laser_frame',0),[250])
        result=evaluate(p,h,raw,requested=requested,measured=.1,baseline=baseline)
        assert result['accepted'] and result['speed_cap_mps']==.1
        pair.append(result)
    assert pair[1]['horizon_m']>=pair[0]['horizon_m']
    ROWS.append(dict(case='low_target_request_'+str(requested),before=pair[0],candidate=pair[1]))


def test_large_request_exposes_existing_three_meter_horizon_rejection_before_crawl_cap():
    point=(.35,0.);raw=scan_with_unobserved((point,));pair=[]
    for baseline in (True,False):
        p,h=seeded((point,))
        if baseline:p.guard_memory=BASE.RetainedCommandMemory([point],.4,.3,('laser_frame',0),[250])
        pair.append(evaluate(p,h,raw,requested=3.2,measured=.1,baseline=baseline))
    assert pair[0]['accepted'] and pair[0]['speed_cap_mps']==.1
    assert not pair[1]['accepted'] and pair[1]['reason']=='memory_command_braking_horizon_unavailable'
    ROWS.append(dict(case='outside_current_speed_scope_3p2',before=pair[0],candidate=pair[1]))


def test_actual_guard_control_allows_side_memory_and_stops_for_current_raw():
    for obstacle in (False,True):
        points=((.06,-.38),);p,h=seeded(points);raw=scan_with_unobserved(points)
        if obstacle:raw.ranges[250]=.05
        node,out,_=guard_node(raw,p,h,speed=.8)
        control,_=fixture.control_method();control(node)
        assert out[-1].drive.speed==(0. if obstacle else .8) and p.latched
        assert len(p.guard_memory.points)==1


@pytest.fixture(scope='session',autouse=True)
def save_rows():
    yield
    (A/'cases.json').write_text(json.dumps(ROWS,ensure_ascii=False,indent=2),encoding='utf-8')
