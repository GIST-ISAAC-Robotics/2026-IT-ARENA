"""재생 초기화는 실제 경로 참조 보존과 STOP의 차이를 유지한다."""
from types import SimpleNamespace
import numpy as np
import pytest
from arena_autonomy.follow_trace import replay_reset
from arena_autonomy.local_follow import FollowConfig, LocalFollow


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('reason', ['decision_source_expired', 'STOP', 'RESET', 'sensor_epoch_changed', None])
def test_only_opted_in_source_expiry_preserves_geometry(enabled, reason):
    config = FollowConfig(junction_continuity=True, expiry_route_reference_trial=enabled)
    normal = LocalFollow(config)
    normal.path = np.array([[0., 0.], [1., 0.]])
    normal.path_ref, normal.path_source, normal.path_wall = 4., 3.9, 'right'
    normal.epoch, normal.side, normal.last_ref, normal.last_source = 2, 'left', 4., 3.9
    normal.held = False
    owner = SimpleNamespace(config=config, normal=normal, reset=normal.reset)
    replay_reset(owner, {'reason': reason})
    assert normal.path is None and normal.held and normal.path_wall is None
    if enabled and reason == 'decision_source_expired':
        assert normal.recent_path[1:] == (4., 3.9, 'right')
        assert normal.epoch == 2
        replay_reset(owner, {'reason': 'STOP'})
    assert normal.recent_path is None and normal.epoch is None
