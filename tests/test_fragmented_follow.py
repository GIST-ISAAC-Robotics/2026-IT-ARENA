from dataclasses import replace

import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_scene import LocalScene, SensorRay
from test_local_follow import history
import arena_autonomy.local_follow as lf


def observed(now, *, near=True, far=True, block=False, unknown=False):
    right_x = np.linspace(-.25, 2., 150)
    points = [np.column_stack((right_x, right_x*0-.425))]
    for enabled, a, b in ((near, -.25, .2), (far, .9, 1.8)):
        if enabled:
            x = np.linspace(a, b, 50)
            points.append(np.column_stack((x, x*0+.425)))
    car_x = np.linspace(.3, .5, 20)
    points.append(np.column_stack((car_x, car_x*0+.23)))
    if block:
        points.append(np.column_stack((np.full(20, .3), np.linspace(-.08, .08, 20))))
    rays = []
    for i, p in enumerate(np.vstack(points)):
        direction = p-[.06, 0.]
        direction /= np.linalg.norm(direction)
        rays.append(SensorRay((.06, 0.), tuple(p), now-.1, 'return', 'laser_frame', 0, i, tuple(direction)))
    if unknown:
        rays.append(SensorRay((.06, 0.), None, now-.1, 'unknown', 'laser_frame', 0, len(rays), (1., 0.)))
    return LocalScene(now, now-.1, now-.1, 'laser_frame', 0, tuple(rays))


def settle(mode, **kwargs):
    ctl = LocalFollow(FollowConfig(boundary_evidence_mode=mode))
    h = history()
    rows = [ctl.command(observed(t, **kwargs), h, 0., 'left') for t in (.4, .5, .6, .7)]
    return ctl, rows


def test_default_still_holds_but_selected_candidate_can_use_actual_fragments():
    _, baseline = settle('connected')
    _, candidate = settle('fragmented_candidate')
    assert all(row[0] == 0 for row in baseline)
    assert candidate[-1][0] > 0 and abs(candidate[-1][1]) < 1e-6
    meta = candidate[-1][2]
    assert meta['boundary_selection'] == 'bilateral_observed_fragments'
    assert meta['occupied_points'] == 270 and meta['unfiltered_occupancy']
    evidence = meta['fragmented_boundary_evidence']
    assert not evidence['is_drive_permission'] and evidence['unknown_gaps_m']
    assert not any('source_indices' in part for part in evidence['fragments'])


@pytest.mark.parametrize('kwargs', [{'near': False}, {'far': False}, {'near': False, 'far': False},
                                  {'block': True}, {'unknown': True}])
def test_fragment_candidate_keeps_required_stops(kwargs):
    _, rows = settle('fragmented_candidate', **kwargs)
    assert all(row[0] == 0 for row in rows), rows


def test_two_fresh_boundary_and_release_confirmations_still_required():
    ctl = LocalFollow(FollowConfig(boundary_evidence_mode='fragmented_candidate'))
    h = history()
    scene = observed(.4)
    for t in (.4, .42, .44):
        assert ctl.command(replace(scene, t_ref_s=t), h, 0., 'left')[0] == 0
    assert ctl.pending_count == 1
    assert ctl.command(observed(.5), h, 0., 'left')[0] == 0
    assert ctl.command(observed(.6), h, 0., 'left')[0] > 0


def test_reverse_motion_blocks_candidate_too():
    ctl, _ = settle('fragmented_candidate')
    output = ctl.command(observed(.8), history(), -.01, 'left')
    assert output[0] == 0 and output[2]['reason'] == 'follow_reverse_motion'


def test_config_is_opt_in_and_no_unknown_mode_falls_back():
    assert FollowConfig().boundary_evidence_mode == 'connected'
    with pytest.raises(ValueError, match='boundary evidence'):
        FollowConfig(boundary_evidence_mode='just_opposite_wall')


def test_candidate_runs_after_final_reacquisition_check_fails(monkeypatch):
    original = lf.local_path
    def folded_preferred(points, side, offset):
        path, details = original(points, side, offset)
        if side == 'left' and path is not None:
            # 관측 벽은 길지만 그 끝면 오프셋이 접혀 경로 자체는 짧은 경우.
            details = {**details, 'boundary_observed_ahead_m': 1.2}
        return path, details
    monkeypatch.setattr(lf, 'local_path', folded_preferred)
    _, baseline = settle('connected')
    _, candidate = settle('fragmented_candidate')
    assert baseline[-1][0] == 0
    assert candidate[-1][0] > 0
    assert candidate[-1][2]['boundary_selection'] == 'bilateral_observed_fragments'
