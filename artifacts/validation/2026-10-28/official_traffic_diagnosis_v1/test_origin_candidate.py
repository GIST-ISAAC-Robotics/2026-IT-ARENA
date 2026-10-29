"""거리 원점 수정의 양방향 경계 사례. 주행·안전 성과를 주장하지 않는다."""
from dataclasses import replace
import numpy as np
import pytest
from origin_candidate import build_previous
previous = build_previous()
from arena_autonomy.local_follow import FollowConfig
from origin_candidate import projected_origin_offset, short_prefix_evidence as candidate


@pytest.mark.parametrize('phase', [-.015, -.01, -.005, 0., .005, .01, .015])
@pytest.mark.parametrize('angle', [0., .4, -1.1])
def test_projection_is_independent_of_sampling_phase_and_rotation(phase, angle):
    p = np.column_stack((np.arange(-.12, .13, .03)+phase, np.full(9, .02)))
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    rotated = p @ rotation.T
    nearest = int(np.argmin(np.linalg.norm(rotated, axis=1)))
    assert projected_origin_offset(rotated, nearest) == pytest.approx(p[nearest, 0], abs=1e-12)


def install_paths(monkeypatch, nearest, start, shared=.4, split=False):
    # 한 후보가 20 cm 선을 앞/뒤 표본 때문에 반대로 판정하는 경우를 분리한다.
    common = start+np.arange(0., shared+.001, .05)
    own_x = np.r_[nearest-.04, nearest, .05, .10, .15, common]
    own = np.column_stack((own_x, np.zeros(len(own_x))))
    other = np.column_stack((common, np.zeros(len(common))))
    if split:
        other[2:4, 1] = .2
    info = dict(wall_distance_m=.425, boundary_observed_ahead_m=.6)
    def local_path(points, side, offset):
        return (own, info) if side == 'right' else (other, dict(info, boundary_observed_ahead_m=1.2))
    monkeypatch.setitem(previous.__globals__, 'local_path', local_path)
    monkeypatch.setitem(candidate.__globals__, 'local_path', local_path)
    return replace(FollowConfig(), boundary_agreement_m=.001), own


def test_vertex_behind_origin_no_longer_rejects_supported_short_lead(monkeypatch):
    config, own = install_paths(monkeypatch, -.01, .195)
    old = previous(own, 'right', config)
    new = candidate(own, 'right', config)
    assert not old['supported'] and new['supported']
    assert new['one_sided_lead_in_m'] == pytest.approx(.195)
    assert new['origin_arc_correction_m'] == pytest.approx(-.01)
    np.testing.assert_allclose(new['path'], own)


def test_vertex_ahead_origin_no_longer_accepts_lead_exceeding_limit(monkeypatch):
    config, own = install_paths(monkeypatch, .01, .205)
    assert previous(own, 'right', config)['supported']
    assert not candidate(own, 'right', config)['supported']


@pytest.mark.parametrize('nearest', [-.01, .01])
def test_genuinely_long_one_sided_lead_stays_rejected(monkeypatch, nearest):
    config, own = install_paths(monkeypatch, nearest, .23)
    assert not candidate(own, 'right', config)['supported']


def test_later_support_island_does_not_fill_a_split(monkeypatch):
    config, own = install_paths(monkeypatch, -.01, .195, split=True)
    assert not candidate(own, 'right', config)['supported']


def test_short_shared_span_does_not_gain_support(monkeypatch):
    config, own = install_paths(monkeypatch, -.01, .195, shared=.15)
    assert not candidate(own, 'right', config)['supported']


@pytest.mark.parametrize('kind', ['occupied', 'unknown', 'stale', 'reverse', 'frame', 'fast'])
def test_candidate_does_not_skip_current_obstacle_or_input_failure(monkeypatch, kind):
    from arena_autonomy import corridor_observer
    from test_short_prefix_connection import scene, config
    monkeypatch.setattr(corridor_observer, 'short_prefix_evidence', candidate)
    sample = scene(.4, obstacle=kind == 'occupied', unknown=kind == 'unknown')
    if kind == 'stale':
        sample = replace(sample, t_ref_s=1.)
    if kind == 'frame':
        sample = replace(sample, source_frame_id='wrong',
                         rays=tuple(replace(r, source_frame_id='wrong') for r in sample.rays))
    speed = -.1 if kind == 'reverse' else 2. if kind == 'fast' else 0.
    result = corridor_observer.current_proposal(sample, speed, 'right', 0., .02, config())
    assert not result['point_occupancy_supported'] and result['proposed_speed_mps'] == 0.


def test_candidate_still_confirms_fresh_geometry_and_rechecks_after_fault(monkeypatch):
    from arena_autonomy import corridor_observer
    from test_short_prefix_connection import scene, config
    from test_local_follow import history
    monkeypatch.setattr(corridor_observer, 'short_prefix_evidence', candidate)
    observer = corridor_observer.CorridorObserver(config())
    h = history()
    assert not observer.evaluate(scene(.4), h, 0., 'right')['positive']
    assert not observer.evaluate(scene(.5), h, 0., 'right')['positive']
    assert not observer.evaluate(scene(.5), h, 0., 'right')['positive']
    assert observer.evaluate(scene(.6), h, 0., 'right')['positive']
    assert not observer.evaluate(scene(.3), h, 0., 'right')['positive']
    assert not observer.evaluate(scene(.7), h, 0., 'right')['positive']
