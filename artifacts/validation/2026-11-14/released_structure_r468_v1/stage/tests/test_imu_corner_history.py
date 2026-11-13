"""IMU 편향 실패 당시 실제 코너 경로. 편향 교정이나 새 자유 공간 증거가 아니다."""
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from arena_autonomy.local_follow import FollowConfig, path_agrees, retained_boundary_agreement

ROOT = Path(__file__).resolve().parents[1]
CASES = json.loads((ROOT / 'artifacts/validation/2026-11-11/c1_native_sampling_v1/imu_arc_fixture.json')
                  .read_text(encoding='utf-8'))['captures']


@pytest.mark.parametrize('case', CASES, ids=lambda c: str(c['time_s']))
def test_current_corner_and_carried_path_share_observed_arc(case):
    candidate, reference = np.asarray(case['candidate']), np.asarray(case['reference'])
    cfg = FollowConfig(**case['config'])
    assert case['info']['boundary_observed_ahead_m'] > cfg.boundary_min_ahead_m
    assert case['info']['path_anchor_error_m'] < cfg.boundary_anchor_error_m
    assert not path_agrees(candidate, reference, cfg.boundary_agreement_m)
    assert retained_boundary_agreement(candidate, reference, case['info'], cfg) == 'shared_arc'
    for factor in ([1., 1.], [1., -1.]):
        assert retained_boundary_agreement(candidate*factor, reference*factor, case['info'], cfg) == 'shared_arc'


@pytest.mark.parametrize('fault', ['arc_off', 'piecewise_off', 'different_corner', 'too_short', 'different_model'])
def test_different_or_insufficient_geometry_is_not_released(fault):
    case = CASES[0]
    candidate, reference = np.asarray(case['candidate']), np.asarray(case['reference'])
    cfg, info = FollowConfig(**case['config']), dict(case['info'])
    if fault == 'arc_off': cfg = replace(cfg, corridor_arc_agreement=False)
    if fault == 'piecewise_off': cfg = replace(cfg, piecewise_wall_trial=False)
    if fault == 'different_corner': candidate = candidate + [0., .3]
    if fault == 'too_short': candidate = candidate[:4]
    if fault == 'different_model': info['wall_model'] = 'other'
    assert retained_boundary_agreement(candidate, reference, info, cfg) is None
