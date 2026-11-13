"""RELEASED에서도 실제 구조 가림만 같은 기준으로 취급하는지 검사한다."""
from dataclasses import replace
from types import SimpleNamespace
import json
import math
import numpy as np
import pytest

from test_current_structure_shadow import example, SECTORS
from arena_autonomy.structure_region import released_structure_region


def evaluate(*, scene_change=None, scan_change=None, region_change=None, partial_change=None,
             config_change=None, sensor_change=None):
    scene, scan, q, padding = example(sensor_change)
    region = dict(queries=len(q), unobserved=0, masked_front_samples=0,
                  masked_left=0, masked_right=len(q))
    partial = dict(allowed=True, known_structure=dict(policy='structure_aware_v1', evaluated=True))
    cfg = dict(known_structure_policy='structure_aware_v1', traffic_recovery_trial=True,
               known_structure_sectors_json=json.dumps(SECTORS))
    region.update(region_change or {});partial.update(partial_change or {});cfg.update(config_change or {})
    if scene_change: scene=replace(scene, **scene_change)
    if scan_change: scan=replace(scan, **scan_change)
    context=dict(queries=q,padding=padding,masked=np.ones(len(q),dtype=bool))
    return released_structure_region(scene, scan, region, context, SimpleNamespace(**cfg), partial)


def test_declared_shadow_does_not_change_selector_history_or_fill_unknown():
    result=evaluate()
    assert result['accepted']
    assert not result['unknown_filled'] and not result['selector_state_changed']
    assert not result['source_timestamp_renewed'] and result['independent_guard_required']


@pytest.mark.parametrize('change',[dict(unobserved=1),dict(masked_front_samples=1),dict(queries=0)])
def test_unknown_front_or_empty_region_is_not_excused(change):
    assert not evaluate(region_change=change)['accepted']


@pytest.mark.parametrize('change',[dict(known_structure_policy='off'),dict(traffic_recovery_trial=False)])
def test_unselected_structure_policy_preserves_old_result(change):
    assert not evaluate(config_change=change)['accepted']


@pytest.mark.parametrize('change',[dict(allowed=False),dict(known_structure={})])
def test_existing_occupancy_or_memory_rejection_is_not_overridden(change):
    assert not evaluate(partial_change=change)['accepted']


@pytest.mark.parametrize('distance',[.14,math.inf,math.nan])
def test_visible_near_or_missing_neighbor_does_not_become_self_structure(distance):
    assert not evaluate(sensor_change=('left',distance))['accepted']


def test_different_raw_frame_does_not_reuse_structure_permission():
    result=evaluate(scan_change=dict(source_frame_id='other_lidar'))
    assert not result['accepted'] and result['reason']=='released_structure_source_mismatch'
