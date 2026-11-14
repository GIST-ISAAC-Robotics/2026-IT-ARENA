"""기존 단독 호길이 비교를 유지 중인 반대 벽에 연결한다. 점유 허가는 별도다."""
from dataclasses import replace
import numpy as np
import pytest

import arena_autonomy.local_follow as m
from test_boundary_bridge import CONFIG
from test_local_follow import history, scene


def corner(exit_length):
    theta = np.linspace(0., np.pi/2, 32)
    turn = .3*np.column_stack((np.sin(theta), 1.-np.cos(theta)))
    tail = np.column_stack((np.full(50, .3), np.linspace(.32, exit_length, 50)))
    return np.vstack((turn, tail))


def setup_case(monkeypatch, fault=None, mirror=1):
    cfg = m.FollowConfig(**CONFIG, boundary_bridge=True, junction_continuity=True,
                        piecewise_wall_trial=True, corridor_arc_agreement=True)
    if fault == 'arc_off':
        cfg = replace(cfg, corridor_arc_agreement=False)
    if fault == 'piecewise_off':
        cfg = replace(cfg, piecewise_wall_trial=False)
    c = m.LocalFollow(cfg)
    candidate, carried = corner(2.5)*[1, mirror], corner(1.5)*[1, mirror]
    assert not m.path_agrees(candidate, carried, cfg.boundary_agreement_m)
    assert m.path_agrees_arc(candidate, carried, cfg.boundary_agreement_m)
    side = 'left' if mirror == 1 else 'right'
    other = 'right' if mirror == 1 else 'left'
    c.path, c.path_ref, c.path_source = carried, .9, .7
    c.path_wall, c.side, c.epoch, c.last_source = other, side, 0, .7
    c.held = False
    if fault == 'no_memory': c.path = None
    if fault == 'expired': c.path_source = -1.
    if fault == 'other_identity': c.path_wall = side
    if fault == 'different_corner': candidate = candidate+[0., .3]
    detail = dict(reason='local_path', wall_model='piecewise_corner',
                  boundary_observed_ahead_m=2., path_anchor_error_m=.01,
                  wall_distance_m=cfg.wall_offset_m, boundary_samples=80,
                  path_points=candidate.tolist())
    if fault == 'short': detail['boundary_observed_ahead_m'] = .3
    if fault == 'anchor': detail['path_anchor_error_m'] = .2
    if fault == 'model': detail['wall_model'] = 'other'
    def observed(points, wall_side, *args, **kwargs):
        if wall_side == other:
            return candidate.copy(), dict(detail)
        return None, dict(reason='wall_segment_short', boundary_observed_ahead_m=0.)
    monkeypatch.setattr(m, 'local_path', observed)
    return c, side


@pytest.mark.parametrize('mirror', [1, -1])
def test_same_retained_opposite_corner_refreshes_without_x_window(monkeypatch, mirror):
    c, side = setup_case(monkeypatch, mirror=mirror)
    result = c._baseline_command(scene(.9), history(), .35, side)
    assert result[2]['boundary_selection'] == 'current_retained_wall_refresh'
    assert result[2]['retained_boundary_agreement'] == 'shared_arc'
    assert c.path_source == pytest.approx(.8)


@pytest.mark.parametrize('fault', ['arc_off', 'piecewise_off', 'no_memory', 'expired',
                                  'other_identity', 'different_corner', 'short', 'anchor', 'model'])
def test_existing_evidence_requirements_remain(monkeypatch, fault):
    c, side = setup_case(monkeypatch, fault)
    result = c._baseline_command(scene(.9), history(), .35, side)
    assert result[2].get('boundary_selection') != 'current_retained_wall_refresh'


def test_new_front_obstacle_still_stops_after_path_refresh(monkeypatch):
    c, side = setup_case(monkeypatch)
    result = c._baseline_command(scene(.9, obstacle_x=.10), history(), .35, side)
    assert result[2]['boundary_selection'] == 'current_retained_wall_refresh'
    assert result[0] == 0
    assert result[2]['reason'] == 'follow_gap_hold'
