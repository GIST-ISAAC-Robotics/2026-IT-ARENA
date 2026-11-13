"""새 후보 분류/실제 부모 제출 경계만 검사. ROS/장치/주행 없음."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from arena_autonomy.race_runtime_policy.crop_current_scope import (
    classified_crop, partial_retry_scope, same_scan_scope)
from arena_autonomy.runtime_dual_process import plan_reference
from arena_autonomy.runtime_dual_state import RuntimeDualProcess, current_request

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def record():
    return json.loads((ROOT/'inputs/representative.json').read_bytes())


@pytest.fixture
def header():
    return json.loads((ROOT/'inputs/header.json').read_bytes())


def owned(record, enabled=True):
    return current_request(record['plan_output'], record['plan_input'], record['current'],
                           retained=record['retained'], crop_current_selection=enabled)


def scope(data, enabled=True):
    return same_scan_scope(data, NS(crop_edge_component_trial=True), registered_environment=enabled)


def test_same_scan_compact_contract_and_no_input_mutation(record):
    before = copy.deepcopy(record); data = owned(record)
    assert scope(data)
    assert data['submitted'] == plan_reference(record['plan_input'])
    assert set(data['submitted']['scan']['measurements']) == {'source_epoch'}
    assert data['crop_current_input']['same_scan'] is True
    assert 'ranges_m' not in json.dumps(data['crop_current_input'])
    assert record == before


@pytest.mark.parametrize('field', ['ranges_m','angles_rad','acquired_at_s','source_frame_id',
    'source_epoch','time_basis','range_min_m','range_max_m'])
def test_whole_scan_metadata_difference_recomputed_at_parent(record, field):
    raw = record['current']['scan']['measurements']
    value = raw[field]
    if isinstance(value, list): value[0] = 0.123
    elif isinstance(value, str): raw[field] = value+'_changed'
    else: raw[field] = value+1
    data = owned(record)
    assert data['crop_current_input']['same_scan'] is False
    assert not scope(data)


@pytest.mark.parametrize('side,key,value', [
    ('plan_input','timing_verified',False), ('current','timing_verified',False),
    ('plan_input','timing_policy','host_estimated_v1'), ('current','timing_policy','host_estimated_v1')])
def test_original_timing_is_preserved_and_rejected(record, side, key, value):
    record[side][key] = value; data = owned(record)
    assert not scope(data)
    if side == 'plan_input': assert data['crop_current_input']['submitted_'+key] == value
    assert record[side][key] == value


@pytest.mark.parametrize('fault', ['false_allowed','general_side','wrong_scope','empty_members',
    'nonadjacent_members','bool_index','forward_reason','latch','memory','wrong_mode'])
def test_false_or_other_plan_markers_do_not_select(record, fault):
    normal = record['plan_output'][2]['normal']; partial = normal['partial_observation']
    component = partial['crop_edge_component']
    if fault == 'false_allowed': partial['allowed'] = False
    elif fault == 'general_side': partial.pop('crop_edge_component')
    elif fault == 'wrong_scope': component['scope'] = 'fully_observed'
    elif fault == 'empty_members': component['source_indices'] = []
    elif fault == 'nonadjacent_members': component['source_indices'][1] += 100
    elif fault == 'bool_index': component['source_indices'][0] = True
    elif fault == 'forward_reason': partial['reason'] = 'bounded_forward_missing'
    elif fault == 'latch': partial['latch_reason'] = 'known_obstacle_latched'
    elif fault == 'memory': normal['boundary_from_memory'] = True
    elif fault == 'wrong_mode': normal['selector_mode'] = 'GUARD'
    assert not classified_crop(record['plan_output']) and not scope(owned(record))


def test_off_and_fully_observed_preserve_old_selection(record):
    data = owned(record, False)
    assert data == dict(output=record['plan_output'], submitted=plan_reference(record['plan_input']),
                        current=record['current'], retained=record['retained'])
    assert not scope(data) and not scope(owned(record), False)
    assert not same_scan_scope(owned(record), NS(crop_edge_component_trial=False), registered_environment=True)
    for enabled in (False, True):
        assert partial_retry_scope(dict(allowed=True, reason='fully_observed'), crop_current_scope=enabled)
        assert not partial_retry_scope(dict(allowed=False, reason='fully_observed'), crop_current_scope=enabled)
    assert not partial_retry_scope(dict(allowed=True, reason='bounded_side_missing'))


class FakeLane:
    def __init__(self, stage, header, **kwargs):
        self.stage = stage; self.inflight = None; self.requests = []
        self.connection = NS(poll=lambda timeout=0: False)
    def offer(self, data, generation, sequence, reset_reason=None):
        self.requests.append(copy.deepcopy(data)); self.inflight = dict(generation=generation, sequence=sequence)
    def close(self): return dict(stage=self.stage, forced=False, alive=False, exitcode=0)


@pytest.mark.parametrize('crop,corner,same', [(False,True,True),(True,False,True),(True,True,True),(True,True,False)])
def test_actual_parent_offer_uses_owned_pair_and_leaves_inputs_unchanged(record, header, crop, corner, same):
    enabled = crop and corner
    header['follow_config']['crop_edge_component_trial'] = crop
    header['current_corner_budget'] = 'r396_v1' if corner else 'off'
    if not same: record['current']['scan']['measurements']['range_max_m'] += 1
    before = copy.deepcopy(record)
    parent = RuntimeDualProcess(header, supported=lambda output: True, lane_factory=FakeLane,
                                evaluator_factory=lambda stage, selected: None)
    try:
        parent.generation = record['generation']; parent.sequence = record['sequence']-1
        plan = dict(generation=record['generation'], sequence=record['plan_sequence'],
                    input=copy.deepcopy(record['plan_input']), output=copy.deepcopy(record['plan_output']))
        if record['retained']: parent.retained = plan
        else: parent.saved_plan = plan
        assert parent.offer_next_current(record['current'], now_s=record['current']['reference_s'])
        sent = parent.current.requests[0]
        assert sent == owned(record, enabled)
        assert scope(sent) is (enabled and same)
        assert record == before
        assert parent.current_input['plan']['input'] == record['plan_input']
    finally: parent.close()


def test_same_scan_with_new_reference_and_motion_does_not_reuse_old_pose(record):
    assert record['current']['reference_s'] != record['plan_input']['reference_s']
    record['current']['previous_steering_rad'] += .001
    data = owned(record)
    assert scope(data)
    assert data['current']['previous_steering_rad'] != data['submitted']['previous_steering_rad']
    data['crop_current_input']['current_reference_s'] -= .001
    assert not scope(data)


def test_unverified_raw_scan_cannot_be_promoted_by_equal_content(record):
    for key in ('plan_input','current'): record[key]['scan']['measurements']['time_basis'] = 'unverified_estimate'
    data = owned(record)
    assert data['crop_current_input']['same_scan'] is True
    assert data['crop_current_input']['submitted_time_basis'] == 'unverified_estimate'
    assert not scope(data)
