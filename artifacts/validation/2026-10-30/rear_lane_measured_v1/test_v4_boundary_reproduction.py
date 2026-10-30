"""10/29 오프라인 v4 원본(수정하지 않음)의 목표 상한 경계 반례를 재현해 보존한다.

설정 최대속도가 기본 target_cap=.35보다 낮으면 원본은 invalid_target_cap을 내고 평행 후보가 새 스캔에서도
계속 HOLD한다. 이 파일의 통과는 '원본 실패가 재현됐다'는 뜻이며, 수정본 검증은
tests/test_measured_lane_preference_trial.py가 맡는다.
"""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

import arena_autonomy.pass_release_trial as ps
from arena_autonomy.local_follow import FollowConfig

ROOT = Path(__file__).resolve().parents[4]
V4 = ROOT/'artifacts/validation/2026-10-29/rear_lane_preference_shadow_v4'
CHAIN = dict(partial_observation_policy='bounded_local_v4', boundary_evidence_mode='corner_distance_candidate',
             occupancy_timing_mode='per_ray_candidate', parallel_hold_trial=True, pass_release_trial=True,
             guard_curve_trial=True, traffic_recovery_trial=True, traffic_path_recovery_trial=True,
             corridor_fallback_enabled=True, rear_lane_keep_trial=True)
PATH = [[x/10, 0.] for x in range(11)]


def load(name):
    # v4 파일은 'from parallel_speed import proposal'을 쓰므로 원본 이름으로 그 디렉터리에서 읽는다.
    spec = importlib.util.spec_from_file_location(name, V4/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def v4(monkeypatch):
    speed = load('parallel_speed')
    state = load('parallel_state')
    assert state.proposal is speed.proposal
    monkeypatch.setattr(speed, 'per_ray_padding', lambda *a: np.array([.1]))
    monkeypatch.setattr(speed, 'swept_limit', lambda *a, **k: 1.4)
    monkeypatch.setattr(ps, 'motion_region', lambda *a, **k: dict(distance_m=.6, queries=100, unobserved=0,
        masked_left=0, masked_right=0, masked_tail_extent_m=0., masked_front_samples=0))
    base = (1., 0., {'reason': 'local_follow', 'path_points': PATH, 'swept_clearance_m': 1.4,
                     'partial_observation': dict(allowed=True, reason='fully_observed')})
    yield speed, state, base
    for name in ('parallel_speed', 'parallel_state'):
        sys.modules.pop(name, None)


@pytest.mark.parametrize('maximum', [.2, .3, .34999])
def test_original_v4_rejects_default_cap_below_configured_maximum(v4, maximum):
    speed, state, base = v4
    cfg = FollowConfig(**CHAIN, max_speed_mps=maximum)
    scene = SimpleNamespace(t_ref_s=.1, source_first_s=0., source_epoch=0, occupied_endpoints_xy_m=[[1.4, 0.]])
    result = speed.proposal(scene, base, .5*maximum, 0., .02, cfg)
    assert result == {'eligible': False, 'reason': 'invalid_target_cap', 'unknown_filled': False,
                      'hidden_rear_risk_unresolved': True, 'straight_trial_only': True}
    trial = state.MeasuredParallelTrial(cfg)
    outputs = []
    for index in range(5):
        scene.source_first_s, scene.t_ref_s = .1*index, .1*index+.1
        outputs.append(trial.apply(scene, base, .5*maximum, 0., .02))
    # 새 스캔이 계속 와도 평행 후보는 0명령 HOLD에 머문다(원본 결함).
    assert all(o[0] == 0 and o[2]['reason'] == 'parallel_hold_invalid_target_cap' for o in outputs)


def test_original_v4_unaffected_at_completed_one_mps_setting(v4):
    speed, state, base = v4
    cfg = FollowConfig(**CHAIN)
    scene = SimpleNamespace(t_ref_s=.1, source_first_s=0., source_epoch=0, occupied_endpoints_xy_m=[[1.4, 0.]])
    result = speed.proposal(scene, base, .7, 0., .02, cfg)
    assert result['eligible'] and result['target_speed_mps'] <= .35
