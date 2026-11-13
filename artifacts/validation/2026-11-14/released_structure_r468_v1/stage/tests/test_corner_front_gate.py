import math

from arena_autonomy import observation_support as obs
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from probe_corner_front_gate import diagnostic_without_front_prefilter
from test_local_follow import history, scan


def test_independent_function_does_not_patch_operating_function():
    original = obs.sweep_support
    shadow = diagnostic_without_front_prefilter()
    assert obs.sweep_support is original and shadow is not original
    scene = motion_scene(scan(.5, [3.]*500), history(), .635, timing_verified=True)
    args = ([scene], 0., .2, 0., 0., FollowConfig())
    kwargs = dict(protected_sensor_pose=(.06, 0., 0.))
    assert shadow(*args, **kwargs) == original(*args, **kwargs)


def test_removing_front_prefilter_does_not_fill_required_unknown():
    raw = scan(.5, [3.]*500)
    raw.ranges[250] = math.inf
    scene = motion_scene(raw, history(), .635, timing_verified=True)
    args = ([scene], 0., .2, 0., 0., FollowConfig())
    kwargs = dict(protected_sensor_pose=(.06, 0., 0.))
    assert obs.sweep_support(*args, **kwargs)['reason'] == 'current_forward_unobserved'
    result = diagnostic_without_front_prefilter()(*args, **kwargs)
    assert not result['supported'] and result['reason'] == 'sweep_unobserved'
    assert not result['unknown_filled']


def test_other_early_rejections_remain():
    shadow = diagnostic_without_front_prefilter()
    assert shadow([], 0., .2, 0., 0., FollowConfig()) == obs.sweep_support([], 0., .2, 0., 0., FollowConfig())
