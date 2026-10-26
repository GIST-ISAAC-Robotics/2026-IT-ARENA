"""급커브 후보의 제어 연결. 합성 관측/계약 검증이지 실제 코스 주행이 아니다."""
from dataclasses import replace
import math

import numpy as np
import pytest

import arena_autonomy.local_follow as lf
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_scene import LocalScene, SensorRay
from test_corner_evidence import reference, wall
from test_local_follow import history, scene, cast_scene, corridor, box, branch


def contract_scene(now, *, inner=True, block=False, unknown=False, source=None):
    # 경계 선택 흐름을 격리한다. 서로 다른 현재 반환과 실제 점유 유지가 목적이다.
    p=reference()
    tangent=np.gradient(p,axis=0)
    tangent/=np.linalg.norm(tangent,axis=1)[:,None]
    outer=p-.425*np.column_stack((-tangent[:,1],tangent[:,0]))
    points=np.vstack((wall() if inner else np.empty((0,2)),outer))
    if block:
        points=np.vstack((points,np.column_stack((np.full(20,.27),np.linspace(-.1,.1,20)))))
    first=now-.1 if source is None else source
    rays=[]
    for i,point in enumerate(points):
        direction=point-[.06,0.]
        direction/=np.linalg.norm(direction)
        rays.append(SensorRay((.06,0.),tuple(point),first,'return','laser_frame',0,i,tuple(direction)))
    if unknown:
        rays.append(SensorRay((.06,0.),None,first,'unknown','laser_frame',0,len(rays),(1.,0.)))
    return LocalScene(now,first,first,'laser_frame',0,tuple(rays))


@pytest.fixture
def isolated_path(monkeypatch):
    # 외측 fitting의 성공/실패와 새 증거 계약/상태기계를 분리한 검사.
    def fitted(points,side,offset):
        if side=='left':
            return None,{'reason':'wall_segment_short'}
        return reference(),{'reason':'ok','wall_distance_m':.425,'boundary_observed_ahead_m':1.2}
    monkeypatch.setattr(lf,'local_path',fitted)


def test_selected_corner_uses_both_actual_sides_and_preserves_all_occupancy(isolated_path):
    ctl=LocalFollow(FollowConfig(boundary_evidence_mode='corner_anchor_candidate'))
    h=history()
    rows=[ctl.command(contract_scene(t),h,0.,'left') for t in (.4,.5,.6,.7)]
    assert rows[-1][0]>0,rows[-1][2]
    assert rows[-1][2]['boundary_selection']=='bilateral_observed_corner_anchor'
    assert rows[-1][2]['occupied_points']==136
    assert 'source_indices' not in rows[-1][2]['corner_boundary_evidence']
    assert rows[0][0]==rows[1][0]==0


@pytest.mark.parametrize('mode',['connected','fragmented_candidate'])
def test_existing_modes_do_not_silently_adopt_corner_contract(isolated_path,mode):
    ctl=LocalFollow(FollowConfig(boundary_evidence_mode=mode))
    for t in (.4,.5,.6,.7):
        assert ctl.command(contract_scene(t),history(),0.,'left')[0]==0


@pytest.mark.parametrize('kwargs',[{'inner':False},{'block':True},{'unknown':True}])
def test_unobserved_or_blocked_corner_cannot_drive(isolated_path,kwargs):
    ctl=LocalFollow(FollowConfig(boundary_evidence_mode='corner_anchor_candidate'))
    for t in (.4,.5,.6,.7):
        assert ctl.command(contract_scene(t,**kwargs),history(),0.,'left')[0]==0


def test_same_scan_reverse_and_epoch_do_not_bypass_confirmation(isolated_path):
    ctl=LocalFollow(FollowConfig(boundary_evidence_mode='corner_anchor_candidate'))
    h=history()
    for t in (.4,.42,.44):
        assert ctl.command(contract_scene(t,source=.3),h,0.,'left')[0]==0
    assert ctl.pending_count==1
    for t in (.5,.6,.7):
        out=ctl.command(contract_scene(t),h,0.,'left')
    assert out[0]>0
    assert ctl.command(contract_scene(.8),h,-.01,'left')[0]==0
    assert ctl.command(contract_scene(.9),h,0.,'left')[0]==0
    assert ctl.command(contract_scene(1.),h,0.,'left')[0]>0
    fresh=contract_scene(1.1)
    changed=replace(fresh,source_epoch=1,
                    rays=tuple(replace(ray,source_epoch=1) for ray in fresh.rays))
    assert ctl.command(changed,h,0.,'left')[0]==0


def test_inner_anchor_loss_does_not_extend_memory_ttl(isolated_path):
    ctl=LocalFollow(FollowConfig(boundary_evidence_mode='corner_anchor_candidate'))
    h=history()
    for t in (.4,.5,.6,.7):
        ctl.command(contract_scene(t),h,0.,'left')
    before=ctl.path_source
    for t in (.8,.9,1.,1.1):
        out=ctl.command(contract_scene(t,inner=False),h,0.,'left')
    assert out[0]==0 and out[2]['reason']=='follow_boundary_unconfirmed'
    assert ctl.path_source==before and ctl.path is None


@pytest.mark.parametrize('geometry',[corridor()+box(.28,0.,.20,.15),branch(1.),branch(-1.)])
def test_existing_straight_and_branch_decisions_unchanged_without_corner_geometry(geometry):
    old=LocalFollow(FollowConfig(boundary_evidence_mode='fragmented_candidate'))
    new=LocalFollow(FollowConfig(boundary_evidence_mode='corner_anchor_candidate'))
    for t in (.4,.5,.6,.7):
        observed=cast_scene(t,geometry)
        a,b=old.command(observed,history(),0.,'left'),new.command(observed,history(),0.,'left')
        assert a[:2]==b[:2]
        assert a[2]['reason']==b[2]['reason']


def test_counterfactual_is_labeled_and_does_not_mutate_or_promote_trace(tmp_path):
    import copy
    from test_follow_trace import make_trace
    from probe_corner_follow import counterfactual
    _,events=make_trace(tmp_path)
    before=copy.deepcopy(events)
    result=counterfactual(events)
    assert result['calls']==7 and not result['closed_loop_pass'] and not result['driving_authorized']
    assert events==before


def test_corner_source_contract_has_explicit_parent_and_unchanged_previous_bytes():
    import hashlib
    import json
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    contract=json.loads((root/'config/tests/corner_anchor_r0_sha256.json').read_text())
    parent=root/contract['parent_contract']
    assert hashlib.sha256(parent.read_bytes()).hexdigest()==contract['parent_contract_sha256']
    previous=json.loads(parent.read_text())
    successor=json.loads((root/'config/tests/corner_distance_r0_sha256.json').read_text())
    assert successor['parent_contract']=='config/tests/corner_anchor_r0_sha256.json'
    assert hashlib.sha256((root/successor['parent_contract']).read_bytes()).hexdigest()==successor['parent_contract_sha256']
    corridor=json.loads((root/'config/tests/corridor_fallback_r0_sha256.json').read_text())
    assert corridor['parent_contract']=='config/tests/corner_distance_r0_sha256.json'
    assert hashlib.sha256((root/corridor['parent_contract']).read_bytes()).hexdigest()==corridor['parent_contract_sha256']
    recent=json.loads((root/'config/tests/recent_observation_r0_sha256.json').read_text())
    assert recent['parent_contract']=='config/tests/corridor_fallback_r0_sha256.json'
    assert hashlib.sha256((root/recent['parent_contract']).read_bytes()).hexdigest()==recent['parent_contract_sha256']
    followers,parent_name=[],'config/tests/recent_observation_r0_sha256.json'
    for name in ('r0_plus_e3_r0_sha256.json','b3_race_session_r0_sha256.json', 'm3_execution_r0_sha256.json', 'm3_command_selection_r0_sha256.json', 'm3_baseline_release_r0_sha256.json', 'm3_release_gate_r0_sha256.json', 'm3_small_center_r0_sha256.json'):
        follower=json.loads((root/'config/tests'/name).read_text(encoding='utf-8'))
        assert follower['parent_contract']==parent_name
        assert hashlib.sha256((root/parent_name).read_bytes()).hexdigest()==follower['parent_contract_sha256']
        followers.append(follower)
        parent_name='config/tests/'+name
    for group in ('approved_changes','approved_evaluation_scripts'):
        for path,row in contract[group].items():
            assert row['before']==previous[group][path]['after']
            later=successor[group].get(path)
            if later:
                assert later['before']==row['after']
                row=later
            later=corridor[group].get(path)
            if later:
                assert later['before']==row['after']
                row=later
            later=recent[group].get(path)
            if later:
                assert later['before']==row['after']
                row=later
            # 10/24 R0+/B3 successor provenance (old contracts unchanged, parent hashes checked)
            for follower in followers:
                later=follower[group].get(path)
                if later:
                    assert later['before']==row['after']
                    row=later
            assert hashlib.sha256((root/path).read_bytes()).hexdigest()==row['after']
    for path,value in contract['new_dependencies'].items():
        assert hashlib.sha256((root/path).read_bytes()).hexdigest()==value
    for path,value in successor['new_dependencies'].items():
        assert hashlib.sha256((root/path).read_bytes()).hexdigest()==value
    for path,value in corridor['new_dependencies'].items():
        if path in recent['modified_dependencies']:
            change=recent['modified_dependencies'][path]
            assert change['before']==value
            value=change['after']
        assert hashlib.sha256((root/path).read_bytes()).hexdigest()==value
    for path,value in recent['new_dependencies'].items():
        parent_name='config/tests/recent_observation_r0_sha256.json'
        for name in ('recent_observation_precision_r0_sha256.json','recent_observation_cost_r0_sha256.json',
                     'recent_observation_wedge_r0_sha256.json','recent_observation_depth_r0_sha256.json',
                     'recent_observation_transport_r0_sha256.json','recent_observation_window_r0_sha256.json',
                     'recent_observation_origin_r0_sha256.json','recent_observation_query_batch_r0_sha256.json'):
            extension=json.loads((root/'config/tests'/name).read_text())
            assert extension['parent_contract']==parent_name
            assert hashlib.sha256((root/parent_name).read_bytes()).hexdigest()==extension['parent_contract_sha256']
            if path in extension['modified_dependencies']:
                change=extension['modified_dependencies'][path]
                assert change['before']==value
                value=change['after']
            parent_name='config/tests/'+name
        assert hashlib.sha256((root/path).read_bytes()).hexdigest()==value
    for path,change in recent['newly_tracked_changes'].items():
        assert len(change['before'])==64
        assert hashlib.sha256((root/path).read_bytes()).hexdigest()==change['after']
