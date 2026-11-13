"""합성 proof 경계와 원 대표를 변조한 음성 사례. 실센서/원 재생 성공이 아니다."""
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest

from arena_autonomy.local_scene import scene_from_measurements
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.race_runtime_policy.crop_current_proof import check,component_metric
from arena_autonomy.race_runtime_policy.crop_current_candidate import try_from_current_result,DIAGNOSTIC
from arena_autonomy.runtime_dual_state import current_request

ROOT=Path(__file__).resolve().parents[1]


def synthetic(*,changes=None,yaw=0.,members=None):
    angles=np.deg2rad(np.arange(-180.,180.));values=np.full(360,5.)
    members=list(range(279,282)) if members is None else members
    values[members]=math.inf
    for i,value in (changes or {}).items():values[i]=value
    stamps=np.linspace(.9,.99,360)
    raw=MeasurementScan(tuple(values),tuple(angles),tuple(stamps),time_basis='simulation_acquisition')
    scene=scene_from_measurements(raw.ranges_m,angles_rad=raw.angles_rad,acquired_at_s=raw.acquired_at_s,
        t_ref_s=1.,range_min=.05,range_max=12.,source_frame_id='laser_frame',source_epoch=0,
        sensor_xy_m=(0.,0.),sensor_yaw_rad=0.,acquisition_to_ref_se2=[(0.,0.,yaw)]*360,
        rear_half_angle_deg=0.,preserve_sampling=True)
    direction=np.array(scene.rays[280].direction_xy);q=np.array([.12*direction])
    whole=np.vstack((q,[.20,.12]));pad=np.full(360,.02)
    component=dict(source_indices=members,flank_source_indices=[members[0]-1,members[-1]+1])
    return scene,raw,q,pad,component,whole


def proof(args):
    scene,raw,q,pad,component,whole=args
    return check(scene,raw,q,pad,component,all_queries=whole,motion_epoch=0)


def test_same_origin_formula_and_full_component_descending_order():
    args=synthetic();before=proof(args);assert before['accepted']
    metric=before['components'][0];assert metric['origin_diameter_m']==0
    assert metric['transverse_width_bound_m']==pytest.approx(
        2*(before['query_reach_m']+before['maximum_padding_m'])*math.sin(metric['direction_span_rad']/2))
    assert before['query_reach_m']==pytest.approx(math.hypot(.20,.12))
    args[4]['source_indices'].reverse();args[4]['flank_source_indices'].reverse()
    assert proof(args)==before


@pytest.mark.parametrize('fault', ['near_flank','near_occupied','nan','under_range','open','wide','central'])
def test_raw_full_gap_boundaries(fault):
    if fault=='near_flank':args=synthetic(changes={278:.15})
    elif fault=='near_occupied':args=synthetic(changes={278:.12})
    elif fault=='nan':args=synthetic(changes={280:math.nan})
    elif fault=='under_range':args=synthetic(changes={280:.001})
    elif fault=='open':args=synthetic(changes={i:math.inf for i in range(278,360)})
    elif fault=='wide':args=synthetic(members=list(range(276,285)))
    else:args=synthetic(yaw=math.radians(-80))
    result=proof(args);assert not result['accepted'],result
    if fault=='central':assert result['reason']=='current_component_or_flank_central'


def test_partial_plan_component_cannot_authorize_a_whole_current_component():
    args=synthetic();args[4]['source_indices']=[280];args[4]['flank_source_indices']=[279,281]
    result=proof(args);assert not result['accepted'] and result['reason']=='different_current_whole_component'


def test_parallel_directions_with_separated_origins_are_not_zero_width():
    rays=[NS(ray_origin_xy_m=(0.,0.),direction_xy=(1.,0.)),
          NS(ray_origin_xy_m=(0.,.06),direction_xy=(1.,0.))]
    metric=component_metric(rays,[0,1],.2,.02)
    assert metric['direction_span_rad']==0
    assert metric['origin_diameter_m']==metric['transverse_width_bound_m']==.06
    assert metric['transverse_width_bound_m']>.05


@pytest.mark.parametrize('origin,direction', [((math.nan,0.),(1.,0.)),((0.,0.),(2.,0.))])
def test_direct_metric_rejects_invalid_synthetic_geometry(origin,direction):
    with pytest.raises(ValueError):component_metric([NS(ray_origin_xy_m=origin,direction_xy=direction)],[0],.2,.02)


def test_entire_direction_span_includes_interior_not_just_flanks():
    rays=[NS(ray_origin_xy_m=(0.,0.),direction_xy=(math.cos(math.radians(a)),math.sin(math.radians(a))))
          for a in (98.,107.,102.)]
    result=component_metric(rays,[0,1,2],.2,.02)
    assert result['direction_span_rad']==pytest.approx(math.radians(9))
    assert result['direction_span_rad']>math.radians(6)


@pytest.fixture
def record():return json.loads((ROOT/'inputs/representative.json').read_bytes())


@pytest.fixture
def header():return json.loads((ROOT/'inputs/header.json').read_bytes())


@pytest.mark.parametrize('fault', ['finite_front','close_side'])
def test_changed_sensor_negative_with_old_positive_marker_still_stops(record,header,fault):
    # 의도적으로 양쪽 raw를 바꾸고 옛 positive 표식을 남긴 합성 음성 사례다.
    # 새 plan을 재실행한 실제 관측 성공으로 세지 않는다.
    from arena_autonomy.race_runtime_policy.evaluator import Evaluator
    members=set(record['plan_output'][2]['normal']['partial_observation']['crop_edge_component']['source_indices'])
    for snapshot in (record['plan_input'],record['current']):
        raw=snapshot['scan']['measurements']
        for i,angle in enumerate(raw['angles_rad']):
            degrees=math.degrees(angle)
            if (fault=='finite_front' and abs(degrees)<10) or (fault=='close_side' and 80<degrees<125 and i not in members):
                raw['ranges_m'][i]=.12
    data=current_request(record['plan_output'],record['plan_input'],record['current'],
        retained=record['retained'],crop_current_selection=True)
    before=copy.deepcopy(data);worker=Evaluator('current',header)
    try:
        result=worker(data);assert result['speed_mps']==0,result
        assert data==before
    finally:worker.close()


def test_single_attempt_cap_preserves_measured_speed_and_stops_at_measured_limit(record):
    data=current_request(record['plan_output'],record['plan_input'],record['current'],
        retained=record['retained'],crop_current_selection=True)
    before=record['expected'];calls=[]
    cfg=NS(crop_edge_component_trial=True,max_steering_rad=.458)
    def core(output,submitted,current,*unused):calls.append((output[0],current['speed_mps']));return None
    data['current']['speed_mps']=.4
    result=try_from_current_result(before,data,None,None,cfg,registered_environment=True,core_rebase=core)
    assert calls==[(.2,.4)] and result[DIAGNOSTIC]['attempted_targets']==1
    assert result['speed_mps']==0 and result[DIAGNOSTIC]['reason']=='current_crop_core_rejected'
    data['current']['speed_mps']=.61
    assert try_from_current_result(before,data,None,None,cfg,registered_environment=True,core_rebase=core) is before
    assert calls==[(.2,.4)]


def test_existing_positive_is_returned_without_new_attempt(record):
    before={**record['expected'],'speed_mps':.1}
    def forbidden(*args):raise AssertionError('positive baseline must not call core')
    assert try_from_current_result(before,{},None,None,None,registered_environment=True,core_rebase=forbidden) is before
