import pytest

from arena_autonomy.local_scene import scene_from_scan
from g2_rejoin_envelope import reach_gap, last_visible_acquisition, evaluate_future


def test_reachable_set_only_grows_with_age_speed_error():
    base=dict(body_radius_m=.125,speed_bound_mps=.5,elapsed_s=.1)
    gap=reach_gap(.5,**base)
    assert gap==pytest.approx(.295)
    assert reach_gap(.5,**{**base,'elapsed_s':.2}) < gap
    assert reach_gap(.5,**{**base,'speed_bound_mps':1.}) < gap
    assert reach_gap(.5,**base,ego_motion_bound_m=.01) < gap
    assert reach_gap(.1,**base)==0


def test_masked_ray_does_not_refresh_observation():
    scene=scene_from_scan([1.,1.],angle_min=0.,angle_increment=3.141592653589793,
                          range_min=.05,range_max=12.,acquired_at_s=[1.,1.1],t_ref_s=1.1)
    assert last_visible_acquisition(scene,['target','target']) == 1.
    assert last_visible_acquisition(scene,['wall','target']) is None


def test_missing_observation_never_authorizes():
    result=evaluate_future([],1.,None,.5)
    assert result['minimum_gap_lower_m'] is None
    assert not result['conditional_envelope_clear']
    assert not result['driving_authorized']


@pytest.mark.parametrize('value',[-.1,float('nan'),True])
def test_invalid_speed_bounds(value):
    with pytest.raises(ValueError):
        reach_gap(.5,body_radius_m=.125,speed_bound_mps=value,elapsed_s=.1)
