"""이송 생성자 대조: 값·취득 시각·거절 의미를 유지해야 한다."""
import pytest

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.observation_support import transport_scene
from benchmark_scene_transport import direct_constructor_transport, reference_replace_transport
from test_corridor_visibility import scene
from test_linear_observation import unknown_at
from test_local_follow import history


@pytest.mark.parametrize('speed', [-.5, 0., .5])
@pytest.mark.parametrize('now', [.4, .42, .5])
@pytest.mark.parametrize('source', [scene(), unknown_at(90), unknown_at(-90)])
def test_all_fields_match_including_unreturned_and_masked_rays(speed, now, source):
    motion = history(v=speed)
    expected = reference_replace_transport(source, motion, now)
    actual = direct_constructor_transport(source, motion, now)
    assert actual == expected
    assert transport_scene(source, motion, now) == expected
    assert actual.source_first_s == source.source_first_s
    assert actual.source_last_s == source.source_last_s
    assert [r.acquired_at_s for r in actual.rays] == [r.acquired_at_s for r in source.rays]
    assert [r.status for r in actual.rays] == [r.status for r in source.rays]


@pytest.mark.parametrize('function', [reference_replace_transport, transport_scene, direct_constructor_transport])
def test_missing_motion_keeps_failure(function):
    with pytest.raises(MotionUnavailable):
        function(scene(), history(end=.2), .5)


@pytest.mark.parametrize('speed', [0., .5])
def test_nonzero_yaw_history_uses_same_rigid_transform(speed):
    motion = history(v=speed, w=.4)
    expected = reference_replace_transport(scene(), motion, .5)
    assert direct_constructor_transport(scene(), motion, .5) == expected
    assert transport_scene(scene(), motion, .5) == expected
