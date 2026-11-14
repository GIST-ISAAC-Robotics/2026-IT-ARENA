import numpy as np
import pytest
import json
from dataclasses import replace
from pathlib import Path
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.continuous_prefix import capsule_intervals,common_intervals,refine


def capsule(a,b,c,d,r):
    return capsule_intervals(*[np.array(v,dtype=float) for v in (a,b,c,d)],r)


def test_perpendicular_band():
    assert np.asarray(capsule((0,-1),(0,1),(-1,0),(1,0),.2))==pytest.approx(np.array([[.4,.6]]))


def test_inside_and_outside_parallel_segments():
    assert capsule((0,.05),(1,.05),(0,0),(1,0),.08)==[(0.,1.)]
    assert capsule((0,.081),(1,.081),(0,0),(1,0),.08)==[]


def test_endpoint_circle_not_infinite_line():
    result=capsule((-1,0),(2,0),(0,0),(1,0),.1)
    assert np.asarray(result)==pytest.approx(np.array([[.3,.7]]))


@pytest.mark.parametrize('count',[2,36,101])
def test_representation_density_preserves_continuous_distance(count):
    own=np.column_stack([np.linspace(0,1,count),np.zeros(count)])
    other=np.array([[.25,.06],[1,.06]])
    intervals,arc=common_intervals(own,other,.1)
    assert np.asarray(intervals)==pytest.approx(np.array([[.17,1.]]))
    assert arc[-1]==pytest.approx(1.)


def test_disconnected_support_islands_not_joined():
    own=np.array([[0,0],[1,0]])
    other=np.array([[0,0],[.2,0],[.3,1],[.7,1],[.8,0],[1,0]])
    intervals,_=common_intervals(own,other,.01)
    assert len(intervals)==2
    assert intervals[0][1]<.22 and intervals[1][0]>.78


@pytest.mark.parametrize('result',[{'supported':True,'reason':'short_observed_prefix_geometry_only'},
                                  {'supported':False,'reason':'missing_path'},
                                  {'supported':False,'reason':'preferred_anchor_unconfirmed'}])
def test_other_successes_and_rejections_are_not_changed(result):
    assert refine(result,None) is result


SAVED=json.loads((Path(__file__).parent/'fixtures/continuous_prefix_paths.json').read_text(encoding='utf-8'))


@pytest.mark.parametrize('case',SAVED['cases'])
def test_saved_boundary_does_not_depend_on_first_matching_vertex(case):
    config=FollowConfig(**SAVED['follow_config'])
    output=refine(case,config)
    assert output['supported'] and output['short_prefix']
    assert .17<output['one_sided_lead_in_m']<.20
    assert output['shared_span_m']>.25
    assert output['common_distance_bound_m']==.08
    assert not output['unknown_filled'] and not output['opposite_path_used_for_steering']
    # 20cm 조건을 바꾼 것이 아니다. 관측된 공통 시작이 더 먼 조건은 거절한다.
    assert not refine(case,replace(config,length_m=.17))['supported']


def test_successor_preserves_prior_source_and_registers_new_dependency():
    import hashlib,zipfile
    from test_source_provenance_r0_plus_b3 import m3_revision_digest
    root=Path(__file__).resolve().parents[1]
    name='config/tests/continuous_prefix_r1_sha256.json'
    contract=json.loads((root/name).read_text(encoding='utf-8'))
    sha=lambda x:hashlib.sha256(x).hexdigest()
    assert sha((root/contract['parent_contract']).read_bytes())==contract['parent_contract_sha256']
    assert sha((root/contract['before_snapshot']).read_bytes())==contract['before_snapshot_sha256']
    with zipfile.ZipFile(root/contract['before_snapshot']) as archive:
        for group in ('approved_changes','approved_evaluation_scripts'):
            for file,version in contract[group].items():
                assert sha(archive.read(file))==version['before']!=version['after']
                assert m3_revision_digest(file,newer_than=name)==version['after']
    for file,digest in contract['new_dependencies'].items():
        assert m3_revision_digest(file,newer_than=name)==digest
