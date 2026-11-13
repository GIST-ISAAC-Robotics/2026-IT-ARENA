"""새 연결의 상태/시각/양쪽 조향/원 목표와 실패 보존 계약. 물리 증명이 아니다."""
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('r375_test_policy', Path(__file__).with_name('candidate.py'))
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


def make_fixture(monkeypatch):
    output = [.089, .22, dict(handoff_state='FOLLOW', normal=dict(
        selector_mode='RELEASED', selector_choice='parallel', reason='parallel_hold_follow',
        boundary_from_memory=False, partial_observation=dict(allowed=True, reason='fully_observed')))]
    current = dict(speed_mps=.06, reference_s=10., previous_steering_rad=.23, dt_s=.096,
                   scan={'measurements': {}})
    region = dict(queries=3, unobserved=1, masked_left=1, masked_right=0,
                  masked_front_samples=0, masked_tail_extent_m=.005)
    core = dict(speed_mps=.089, steering_rad=.22, reference_s=10., path_source_s=9.88,
                original_reference_s=9.99, path_points=[[0., 0.]],
                rebased_selector_mode='RELEASED', rebased_selector_choice='parallel',
                reason='rebase_current_released_parallel', maximum_body_corner_lateral_m=.009,
                current_side_presence={'left': False, 'right': False})
    before = {**core, 'speed_mps': 0., 'reason': 'route_current_large_motion_region_unobserved',
        'current_route_proof': dict(accepted=False, region=copy.deepcopy(region),
            bounded_current_risk=dict(accepted=False, target_mps=.089,
                proof=dict(policy='guard_low_speed_body_nonreturn_r136', reason='near_gap_neighbor')))}
    proof = dict(accepted=True, unknown_filled=False, residual_hidden_object_risk=True)
    calls = []
    monkeypatch.setattr(policy.prior, 'rebase', lambda *a, **k: before)
    def rebase(*args):
        calls.append(('core', args))
        return core
    monkeypatch.setattr(policy.core_policy, 'rebase', rebase)
    def motion(*args, context, **kwargs):
        calls.append(('motion', args))
        context.update(queries=np.array([[-.04, .08], [-.03, .08], [.2, 0.]]),
            free=np.array([False, False, True]), masked=np.array([False, True, False]), padding=np.array([.115]))
        return region
    monkeypatch.setattr(policy.ps, 'motion_region', motion)
    monkeypatch.setattr(policy.ps, 'lateral_config', lambda cfg: None)
    monkeypatch.setattr(policy, 'MeasurementScan', lambda **kwargs: None)
    def bounded(*args, **kwargs):
        calls.append(('bounded', args, kwargs))
        return proof
    monkeypatch.setattr(policy, 'bounded_body_nonreturn', bounded)
    cfg = SimpleNamespace(length_m=.207, width_m=.15, max_steering_rad=.4578,
                          released_body_gap_trial=True, traffic_recovery_trial=True)
    return dict(output=output, current=current, before=before, core=core, region=region,
                proof=proof, calls=calls, cfg=cfg, submitted={'reference_s': 9.99})


def run(f, **kwargs):
    return policy.rebase(f['output'], f['submitted'], f['current'], None, None, f['cfg'],
                         require_current_region=kwargs.get('require_current_region', True))


def test_preserves_original_target_path_actual_speed_and_both_steerings(monkeypatch):
    f = make_fixture(monkeypatch)
    saved = copy.deepcopy((f['output'], f['current'], f['before']))
    result = run(f)
    assert result['speed_mps'] == .089 and result[policy.DIAGNOSTIC]['applied']
    assert f['calls'][0][1][0] is f['output']  # RELEASED parallel 입력을 변경하지 않는다.
    assert f['calls'][1][1][1:6] == (.06, .089, .22, .23, .096)
    assert f['calls'][2][2] == {'relative_neighbor': True}
    assert f['calls'][2][1][2].tolist() == [[-.04, .08]]
    assert result['current_route_proof']['region']['unobserved'] == 1
    assert (f['output'], f['current'], f['before']) == saved


@pytest.mark.parametrize('case', ['positive', 'other_stop', 'solo', 'guard', 'baseline', 'hold',
    'memory', 'partial', 'target_fast', 'target_stop', 'fast', 'reverse', 'nan', 'bool', 'dt',
    'previous_angle', 'plan_angle', 'fixed_other', 'option_off', 'traffic_off', 'not_required'])
def test_outside_scope_is_identical(monkeypatch, case):
    f = make_fixture(monkeypatch); out = f['output']; n = out[2]['normal']
    if case == 'positive': f['before']['speed_mps'] = .05
    if case == 'other_stop': f['before']['reason'] = 'released_parallel_current_side_present'
    if case in ('solo', 'guard'): n['selector_mode'] = case.upper()
    if case == 'baseline': n['selector_choice'] = 'baseline'
    if case == 'hold': out[2]['handoff_state'] = 'HOLD'
    if case == 'memory': n['boundary_from_memory'] = True
    if case == 'partial': n['partial_observation']['allowed'] = False
    if case in ('target_fast', 'target_stop'): out[0] = .101 if case == 'target_fast' else 0.
    if case in ('fast', 'reverse', 'nan', 'bool'):
        f['current']['speed_mps'] = {'fast': .151, 'reverse': -.001, 'nan': float('nan'), 'bool': True}[case]
    if case == 'dt': f['current']['dt_s'] = .101
    if case == 'previous_angle': f['current']['previous_steering_rad'] = .46
    if case == 'plan_angle': out[1] = .46
    if case == 'fixed_other': f['before']['current_route_proof']['bounded_current_risk']['proof']['reason'] = 'mask_boundary'
    if case == 'option_off': f['cfg'].released_body_gap_trial = False
    if case == 'traffic_off': f['cfg'].traffic_recovery_trial = False
    assert run(f, require_current_region=case != 'not_required') is f['before']
    assert not f['calls']


@pytest.mark.parametrize('case', ['core_stop', 'core_reason', 'new_side', 'excursion', 'speed_changed',
    'angle_changed', 'time_changed', 'source_changed', 'path_changed', 'region_changed',
    'front_mask', 'large_tail', 'proof_rejected'])
def test_current_failures_cannot_gain_permission(monkeypatch, case):
    f = make_fixture(monkeypatch)
    if case == 'core_stop': f['core']['speed_mps'] = 0.
    if case == 'core_reason': f['core']['reason'] = 'rebase_path_source_expired_or_invalid'
    if case == 'new_side': f['core']['current_side_presence'] = {'left': True, 'right': False}
    if case == 'excursion': f['core']['maximum_body_corner_lateral_m'] = .01001
    if case == 'speed_changed': f['core']['speed_mps'] = .09
    if case == 'angle_changed': f['core']['steering_rad'] = .221
    if case == 'time_changed': f['core']['reference_s'] = 10.001
    if case == 'source_changed': f['core']['path_source_s'] = 9.89
    if case == 'path_changed': f['core']['path_points'] = [[.01, 0.]]
    if case == 'region_changed': f['region']['unobserved'] = 0
    if case in ('front_mask', 'large_tail'):
        field, value = ('masked_front_samples', 1) if case == 'front_mask' else ('masked_tail_extent_m', .01001)
        f['region'][field] = value
        f['before']['current_route_proof']['region'][field] = value
    if case == 'proof_rejected': f['proof']['accepted'] = False
    assert run(f)['speed_mps'] == 0.
