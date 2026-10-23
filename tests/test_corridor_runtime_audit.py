from copy import deepcopy
from dataclasses import asdict
import numpy as np
import pytest

from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from audit_corridor_runtime import handoff_check, inspect_events, selection_contract
from test_local_follow import history


def config():
    return FollowConfig(corridor_fallback_enabled=True, boundary_evidence_mode='corner_distance_candidate',
                        occupancy_timing_mode='per_ray_candidate')


def output():
    return [1., .2, dict(corridor_selected=True, underlying_reason='follow_boundary_unconfirmed',
        reason='local_follow_corridor_candidate', corridor_candidate=dict(positive=True, confirmations=2,
            release_count=2, source_first_s=.2), unfiltered_occupancy=True,
        wall_identity_certified=False, boundary_memory_from_corridor=False)]


def test_contract_records_claims_without_claiming_hardware_proof():
    assert all(selection_contract(output(), {'reference_s': .32}, config()).values())


@pytest.mark.parametrize('key,value', [('underlying_reason','follow_gap_hold'),
    ('wall_identity_certified',True), ('boundary_memory_from_corridor',True), ('unfiltered_occupancy',False)])
def test_broken_selected_contract_is_visible(key,value):
    value_out=output()
    value_out[2][key]=value
    assert not all(selection_contract(value_out,{'reference_s':.32},config()).values())


@pytest.mark.parametrize('key,value', [('positive',False),('confirmations',1),('release_count',1),('source_first_s',0.)])
def test_unconfirmed_or_old_selection_is_not_passed(key,value):
    value_out=output()
    value_out[2]['corridor_candidate'][key]=value
    assert not all(selection_contract(value_out,{'reference_s':.32},config()).values())


def previous():
    return dict(seq=1,time_s=.4,source_s=.3,mode='baseline',key=('laser_frame',0,'left'),
                path=np.column_stack((np.linspace(0,1,21),np.zeros(21))))


def test_handoff_uses_measured_motion_not_ground_truth():
    before=previous()
    after={**before,'seq':2,'time_s':.42,'mode':'corridor'}
    result=handoff_check(before,after,history(),config())
    assert result['evaluated'] and result['path_agrees']
    shifted={**after,'path':after['path']+[0,.2]}
    assert handoff_check(before,shifted,history(),config())['path_agrees'] is False


@pytest.mark.parametrize('change,reason',[({'time_s':.7},'previous_path_expired'),
    ({'key':('laser_frame',1,'left')},'frame_epoch_or_side_changed')])
def test_unavailable_handoff_is_not_counted_as_agreement(change,reason):
    before=previous()
    result=handoff_check(before,{**before,'mode':'corridor',**change},history(),config())
    assert not result['evaluated'] and result['path_agrees'] is None and result['reason']==reason


def events():
    return [dict(seq=0,kind='header',data=dict(schema=1,scope='g1_decision_stage_only',
                follow_config=asdict(config()),motion_config=asdict(MotionConfig()))),
            dict(seq=1,kind='call',data=dict(input={'reference_s':.4},
                output=[0.,0.,dict(reason='follow_boundary_unconfirmed',corridor_selected=False,
                                   corridor_candidate={'reason':'long_outer_reference_unavailable'})])),
            dict(seq=2,kind='footer',data=dict(capture_errors=0,failed=0,dropped=0,pending=0,
                                             worker_alive=False,accepted=2,written=2))]


def test_no_selected_calls_is_zero_coverage_not_candidate_success():
    result=inspect_events(events())
    assert result['summary']['selected_calls']==result['summary']['handoffs_evaluated']==0
    assert not result['complete_lap_verified'] and not result['runtime_changed']


@pytest.mark.parametrize('fault',['missing_footer','gap','dropped','trailing','disabled'])
def test_capture_failure_is_not_partial_audit_success(fault):
    source=deepcopy(events())
    if fault=='missing_footer':source.pop()
    if fault=='gap':source[1]['seq']=5
    if fault=='dropped':source[-1]['data']['dropped']=1
    if fault=='trailing':source.append(source[-1])
    if fault=='disabled':source[0]['data']['follow_config']['corridor_fallback_enabled']=False
    with pytest.raises(ValueError):inspect_events(source)


def test_enabled_recorded_calls_are_audited_without_replaying_controller_state(tmp_path):
    from arena_autonomy.follow_trace import FollowTrace
    from arena_autonomy.local_follow import LocalFollow
    from arena_autonomy.scene_motion import motion_scene
    from compare_corridor_closed_loop import scan_at
    import json
    cfg=config()
    ctl=LocalFollow(cfg)
    h=history()
    path=tmp_path/'decisions.jsonl'
    recorder=FollowTrace(path,h.config,cfg)
    # 고정 자세의 순차 취득 합성 입력이다. 실차/공식 코스의 검증은 아니다.
    truth=np.column_stack((np.arange(0,1.01,.01),np.zeros((101,5))))
    count=0
    for now in (.4,.5,.6,.62,.7):
        raw=scan_at(now,truth,1,'open')
        scene=motion_scene(raw,h,now,timing_verified=True,epoch=0,rear_half_angle_deg=30.)
        out=ctl.command(scene,h,0.,'left',.2,.02)
        count+=bool(out[2].get('corridor_selected'))
        recorder.record_call(raw,h,now,0,0.,'left',.2,.02,True,30.,out)
    recorder.close()
    report=inspect_events(json.loads(line) for line in path.read_text().splitlines())
    assert count>0 and report['summary']['selected_calls']==count
    assert report['summary']['selected_contract_failures']==0
    assert report['summary']['depth_evaluated']==count
    assert not report['complete_lap_verified']
