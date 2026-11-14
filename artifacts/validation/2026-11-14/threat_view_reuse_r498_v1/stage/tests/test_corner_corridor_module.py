"""미연결 회랑 모듈의 입력/모호함 계약 및 보존 prototype과의 제한 동등성."""
import ast
from pathlib import Path
import numpy as np
import pytest

import arena_autonomy.corner_corridor as module
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import MAX_RAYS
from probe_corner_corridor import paired_geometry
from test_corner_corridor_shadow import ring_scene
from probe_short_corner_support import compact_circle_fixture


@pytest.mark.parametrize('side,sign', [('left',1),('right',-1)])
@pytest.mark.parametrize('radius', [.075,.15,.30,.6])
def test_pure_geometry_matches_preserved_prototype(side, sign, radius):
    scene = ring_scene(radius, sign)
    points = np.asarray(scene.occupied_endpoints_xy_m)
    before = points.copy()
    old = paired_geometry(points, side, FollowConfig())
    new = module.corridor_evidence(points, side, FollowConfig())
    assert new['supported'] == old['geometry_supported']
    assert new['reason'] == old['reason']
    if new['supported']:
        np.testing.assert_array_equal(new['path'], old['path'])
        assert new['common_prefix_m'] == old['common_prefix_m']
        assert [w['source_point_indices'] for w in new['witnesses']] == [w['source_point_indices'] for w in old['witnesses']]
    np.testing.assert_array_equal(points, before)
    for field in ('wall_identity_certified','memory_refresh_authorized','free_area_certified','driving_authorized',
                  'unknown_gaps_filled','occupancy_evaluated','temporal_confirmation_evaluated','runtime_connected'):
        assert new[field] is False


@pytest.mark.parametrize('points', [None, 'not_points', [[1.,2.,3.]]*30, [[float('nan'),0.]]*30,
                                  [[1.,2.]]*29, [[1.,2.]]*(MAX_RAYS+1)])
def test_bad_input_rejected_without_fabricating_evidence(points):
    result = module.corridor_evidence(points, 'left', FollowConfig())
    assert not result['supported'] and result['path'] is None
    assert not result['driving_authorized']


def test_short_surface_never_acquires_wall_identity(monkeypatch):
    points, path, _ = compact_circle_fixture()
    points = np.vstack((points, np.column_stack((np.linspace(-1.,1.,40), np.full(40,-2.)))))
    monkeypatch.setattr(module, 'local_path', lambda *args: (path, dict(boundary_observed_ahead_m=1.2, wall_distance_m=.425)))
    result = module.corridor_evidence(points, 'left', FollowConfig())
    assert result['supported'] and result['inner_identity'] == 'unknown'
    assert max(w['observed_extent_lower_bound_m'] for w in result['witnesses']) < .20
    assert result['inner_semantic_hypotheses'] == ['boundary_surface','other_occupied_object']
    assert not result['memory_refresh_authorized']


def test_module_does_not_import_tools_ros_or_ground_truth():
    source = Path(module.__file__).read_text()
    tree = ast.parse(source)
    imports = [node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)]
    assert imports == ['arena_autonomy.local_path', 'arena_autonomy.local_scene']
    assert not any(token in source for token in ('rclpy', 'gazebo', 'probe_corner', 'create_publisher'))
