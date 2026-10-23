import numpy as np
import pytest
from probe_target_velocity_bounds import DiskClearanceModel
from probe_target_center_envelope import CenterEnvelope,center_bounds
from test_observation_target_extent import beam
from test_target_velocity_bounds import CFG,MOTION,META


def test_center_can_move_so_edge_not_center_hits_query():
    model=DiskClearanceModel(beam(stamp=.6),(1.,0.),MOTION,CFG,META)
    envelope=CenterEnvelope([model],0.)
    assert envelope.distances((0.,0.)).min()==0.
    assert envelope.distances((0.,.05)).min()==pytest.approx(.05)
    result=center_bounds(envelope,.05)
    assert not result['all_centers_refuted']
    assert result['score_upper_m'] >= .05-1e-12


def test_age_allows_all_bounded_displacement_not_fresh_free_space():
    model=DiskClearanceModel(beam(stamp=.5),(1.,0.),MOTION,CFG,META)
    old=CenterEnvelope([model],.5).distances((0.,0.))
    assert old[0]==pytest.approx(.05)
    assert CenterEnvelope([model],0.).distances((0.,0.))[0]==0.


@pytest.mark.parametrize('seed',range(8))
def test_per_constraint_envelope_bounds_adversarial_past_positions(seed):
    rng=np.random.default_rng(seed)
    model=DiskClearanceModel(beam(stamp=.5),(1.,.1),MOTION,CFG,META)
    envelope=CenterEnvelope([model],.5)
    offset=rng.uniform(-.1,.1,2)
    # 현재 중심의 각 과거 위치는 |v|≤.5인 등속 사례를 모두 포함해야 한다.
    shifted=DiskClearanceModel(beam(stamp=.5),model.center+offset,MOTION,CFG,META)
    for angle in rng.uniform(-np.pi,np.pi,20):
        v=.5*np.array([np.cos(angle),np.sin(angle)])
        assert np.all(shifted.distances(v)<=envelope.distances(offset)+1e-12)


def test_two_observed_sides_can_rule_out_all_centers():
    # 두 측정선 y=±.1. 질의=(1,0)을 덮는 반경 .15 원판은 둘을 모두 피할 수 없다.
    models=[DiskClearanceModel(beam(origin=(0.,y),stamp=.6),(1.,0.),MOTION,CFG,META)
            for y in (-.1,.1)]
    result=center_bounds(CenterEnvelope(models,0.),.15)
    assert result['all_centers_refuted']
    assert result['score_upper_m'] < .15


def test_unknown_has_no_ray_distance_constraint():
    model=DiskClearanceModel(beam(status='unknown'),(1.,0.),MOTION,CFG,META)
    result=center_bounds(CenterEnvelope([model],.5),.05)
    assert result['unresolved']
    assert result['unrefuted_center_sample_found']


def test_cell_budget_cannot_be_reported_as_proof():
    model=DiskClearanceModel(beam(stamp=.6),(1.,0.),MOTION,CFG,META)
    result=center_bounds(CenterEnvelope([model],0.),.05,max_cells=1)
    assert not result['tolerance_met']
    assert result['unresolved']
