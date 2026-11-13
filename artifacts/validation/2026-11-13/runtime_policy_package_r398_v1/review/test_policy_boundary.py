"""이관/시각 정책의 제한 반례. 동역학·실시간·하드웨어 검사가 아니다."""
import ast
from contextlib import contextmanager
import copy
import json

import pytest

from review_support import (BASE, ROOT, PACKAGE, corpus, selected_header, estimated_call,
                            encoded, digest, state_value, OriginalEvaluator)
from arena_autonomy.race_runtime_policy.evaluator import Evaluator


@pytest.fixture(scope='module', autouse=True)
def frozen_inputs():
    from compare_spawn import verify_manifest
    before = verify_manifest()
    yield
    assert verify_manifest() == before


@pytest.fixture(scope='module')
def saved():
    header, plans, currents = corpus()
    return header, plans, currents


@contextmanager
def current_evaluator(header, *, original=False, estimated=False):
    configured = selected_header(header, 'python', 'host_estimated_v1' if estimated else 'verified_only')
    evaluator = OriginalEvaluator('current', configured) if original else Evaluator('current', configured)
    try:
        yield evaluator
    finally:
        evaluator.close()


def current_case(saved, index=0, *, estimated=False):
    row = copy.deepcopy(saved[2][index]['data'])
    if estimated:
        row['current'] = estimated_call(row['current'])
    return row


def test_ledger_function_bodies_only_intended_geometry_change():
    ledger = json.loads((BASE / 'source_ledger.json').read_bytes())
    assert len(ledger) == 17
    old_gate = "    if (not current['timing_verified'] or not source_valid(scene)\n"
    new_gate = ("    estimated = (current.get('timing_policy') == 'host_estimated_v1'\n"
                "                 and getattr(history, 'scan_time_policy', None) == 'host_estimated_v1'\n"
                "                 and raw['measurements']['time_basis'] == 'unverified_estimate')\n"
                "    if (not (current['timing_verified'] or estimated) or not source_valid(scene)\n")
    for relative, record in ledger.items():
        source, staged = ROOT / record['source'], PACKAGE / relative
        assert digest(source) == record['source_sha256']
        assert digest(staged) == record['staged_sha256']
        before, after = source.read_text(encoding='utf8'), staged.read_text(encoding='utf8')
        if relative == 'retained_recheck/_geometry.py':
            assert before.count(old_gate) == after.count(new_gate) == 1
            assert before.replace(old_gate, new_gate) == after
            after = after.replace(new_gate, old_gate)
        def definitions(text):
            return {node.name: ast.dump(node, include_attributes=False) for node in ast.parse(text).body
                    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))}
        assert definitions(before) == definitions(after), relative
        assert 'importlib' not in after and 'sys.path' not in after, relative


def test_explicit_policy_and_time_selection_required(saved):
    header = selected_header(saved[0])
    for changes in ({'runtime_policy': 'original_v1'}, {'runtime_policy': None},
                    {'runtime_time_policy': 'automatic'}):
        with pytest.raises(ValueError):
            Evaluator('current', {**header, **changes})
    with pytest.raises(ValueError):
        Evaluator('unknown', header)


def test_raw_converted_history_and_scene_numerically_identical(saved):
    from arena_autonomy.follow_trace import restore_call
    from arena_autonomy.lidar_motion import MotionConfig
    from arena_autonomy.scene_motion import motion_scene
    verified = current_case(saved)['current']
    estimated = estimated_call(verified)
    motion = MotionConfig(**saved[0]['motion_config'])
    outputs = []
    for raw in (verified, estimated):
        info, scan, history = restore_call(raw, motion)
        for name in ('wheels', 'gyro'):
            assert getattr(history, name).times == raw['history'][name]['times']
            assert getattr(history, name).values == raw['history'][name]['values']
        outputs.append(state_value(motion_scene(scan, history, info['reference_s'],
            timing_verified=info['timing_verified'], epoch=info['epoch'],
            rear_half_angle_deg=info['rear_half_angle_deg'],
            timing_policy=info.get('timing_policy', 'verified_only'))))
    assert encoded(outputs[0]) == encoded(outputs[1])
    assert estimated['timing_verified'] is False


@pytest.mark.parametrize('index', [0, 123])
def test_same_values_time_policy_keeps_positive_or_unknown_rejection(saved, index):
    verified, estimated = current_case(saved, index), current_case(saved, index, estimated=True)
    with current_evaluator(saved[0], original=True) as old:
        expected = old(verified)
    with current_evaluator(saved[0], estimated=True) as new:
        result = new(estimated)
    assert encoded(result) == encoded(expected)
    assert (result['speed_mps'] > 0) is (index == 0)
    if index == 123:
        assert result['current_route_proof']['region']['unobserved'] > 0
    assert estimated['current']['timing_verified'] is False


@pytest.mark.parametrize('original', [True, False])
def test_simulation_false_verified_only_still_rejects(saved, original):
    data = current_case(saved)
    data['current']['timing_verified'] = False
    with current_evaluator(saved[0], original=original) as evaluator:
        result = evaluator(data)
    assert result['speed_mps'] == 0
    assert result['input_error'] == 'scan_timing_unverified'
    assert result['runtime_authorized'] is False


@pytest.mark.parametrize('estimated', [False, True])
@pytest.mark.parametrize('fault', ['stale_scan', 'missing_wheels', 'all_unknown', 'changed_epoch'])
def test_current_rejections_survive_time_policy(saved, estimated, fault):
    data = current_case(saved, estimated=estimated)
    current = data['current']
    if fault == 'stale_scan':
        current['reference_s'] += 1.
    elif fault == 'missing_wheels':
        current['history']['wheels'] = dict(times=[], values=[])
    elif fault == 'all_unknown':
        ranges = current['scan']['measurements']['ranges_m']
        ranges[:] = [{'nonfinite': 'nan'} for _ in ranges]
    else:
        current['epoch'] += 1
    before = encoded(data)
    with current_evaluator(saved[0], estimated=estimated) as evaluator:
        result = evaluator(data)
    assert encoded(data) == before
    assert result['speed_mps'] == 0 and result['runtime_authorized'] is False
    if fault == 'stale_scan':
        assert result['input_error'] == 'scan_stale'
    elif fault == 'missing_wheels':
        assert result['input_error'] == 'motion_waiting'
    elif fault == 'changed_epoch':
        assert result['reason'] == 'rebase_epoch_or_side_changed'


def test_estimate_is_not_silently_selected_or_relabelled(saved):
    data = current_case(saved, estimated=True)
    with current_evaluator(saved[0]) as strict:
        with pytest.raises(ValueError, match='differs'):
            strict(data)
    data['current'].pop('timing_policy')
    with current_evaluator(saved[0], estimated=True) as estimated:
        with pytest.raises(ValueError, match='differs'):
            estimated(data)
    data['current']['timing_policy'] = 'host_estimated_v1'
    data['current']['scan']['measurements']['time_basis'] = 'simulation_acquisition'
    with current_evaluator(saved[0], estimated=True) as estimated:
        result = estimated(data)
    assert result['speed_mps'] == 0
    assert result['input_error'] == 'scan_timing_policy_basis_mismatch'


def test_geometry_requires_matching_history_policy(saved):
    from arena_autonomy.follow_trace import restore_call
    from arena_autonomy.scene_motion import motion_scene
    from arena_autonomy.race_runtime_policy.retained_recheck import _geometry
    data = current_case(saved, estimated=True)
    with current_evaluator(saved[0], estimated=True) as evaluator:
        info, raw, history = restore_call(data['current'], evaluator.motion)
        scene = motion_scene(raw, history, info['reference_s'], timing_verified=False,
            timing_policy='host_estimated_v1', epoch=info['epoch'],
            rear_half_angle_deg=info['rear_half_angle_deg'])
        history.scan_time_policy = 'verified_only'
        result = _geometry.rebase(data['output'], data['submitted'], data['current'], scene,
                                  history, evaluator.follow.config)
    assert result['speed_mps'] == 0
    assert result['reason'] == 'rebase_current_input_invalid'


def patch_targets():
    from arena_autonomy.local_follow import LocalFollow
    from arena_autonomy.pass_release_trial import PassReleaseSelector
    from arena_autonomy.bypass_handoff import BypassHandoffFollow
    return LocalFollow._baseline_command, PassReleaseSelector._select, BypassHandoffFollow.command


def test_direct_close_restores_plan_patches_and_is_idempotent(saved):
    before = patch_targets()
    evaluator = Evaluator('plan', selected_header(saved[0]))
    try:
        assert all(a is not b for a, b in zip(before, patch_targets()))
    finally:
        evaluator.close()
    assert all(a is b for a, b in zip(before, patch_targets()))
    evaluator.close()
    assert all(a is b for a, b in zip(before, patch_targets()))


def test_failed_controller_construction_restores_plan_patches(saved):
    before = patch_targets()
    header = selected_header(saved[0])
    header['follow_config']['not_a_real_field'] = 1
    with pytest.raises(TypeError):
        Evaluator('plan', header)
    assert all(a is b for a, b in zip(before, patch_targets()))


def test_plan_false_verified_only_returns_zero(saved):
    header = selected_header(saved[0])
    data = copy.deepcopy(saved[1][0]['data'])
    data['timing_verified'] = False
    evaluator = Evaluator('plan', header)
    try:
        result = evaluator(data)
    finally:
        evaluator.close()
    assert result[0] == 0 and result[2]['input_error'] == 'scan_timing_unverified'
