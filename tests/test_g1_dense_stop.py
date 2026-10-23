"""실행 STOP 완료는 저속 상태값/ACK만으로 통과할 수 없다."""
import hashlib
import json
import math
from pathlib import Path

import pytest

import g1_traffic_eval as ev
from test_g1_traffic_eval import verdict_base


def poses(speed=0.):
    return [(1.+i*.002, speed*i*.002, 0., 0.) for i in range(301)]


def test_full_dense_stop_has_existing_limit_and_hold():
    result = ev.stop_pose_evidence(poses(), 1.6, .5)
    assert result['passed'] and result['complete']
    assert result['segment_count'] == 250
    assert result['required_last_s']-result['required_first_s'] == .5
    assert result['maximum_allowed_gap_s'] == .05


@pytest.mark.parametrize('kind', ['moving', 'truncated', 'gap', 'empty', 'one_pose'])
def test_zero_diagnostics_cannot_substitute_for_pose_evidence(kind):
    data = poses(.02 if kind == 'moving' else 0.)
    if kind == 'truncated': data = data[:-80]
    elif kind == 'gap': data = [p for p in data if not 1.2 < p[0] < 1.4]
    elif kind == 'empty': data = []
    elif kind == 'one_pose': data = data[-1:]
    evidence = ev.stop_pose_evidence(data, 1.6, .5)
    assert not evidence['passed']
    report = verdict_base({'hold': True}, 1, 1)
    report['stop']['pose_evidence'] = evidence
    result = ev.finalize_verdict(report, 'g1')
    assert result['stop_checks']['model_stop_stable']  # 옛 진단상 stable=true여도
    assert not result['passed'] and not result['stop_checks']['dense_pose_stop_evidence']


def test_missing_evidence_is_not_old_report_success_upgraded():
    report = verdict_base({'hold': True}, 1, 1)
    del report['stop']['pose_evidence']
    assert not ev.finalize_verdict(report, 'g1')['stop_checks']['dense_pose_stop_evidence']


@pytest.mark.parametrize('fault', ['duplicate', 'backwards', 'nan', 'short', 'zero_hold', 'nan_end'])
def test_invalid_data_is_explicit_failure(fault):
    data, end, hold = poses(), 1.6, .5
    if fault == 'duplicate': data.append(data[-1])
    elif fault == 'backwards': data = list(reversed(data))
    elif fault == 'nan': data[40] = (data[40][0], math.nan, 0., 0.)
    elif fault == 'short': data[40] = data[40][:3]
    elif fault == 'zero_hold': hold = 0.
    elif fault == 'nan_end': end = math.nan
    with pytest.raises(ValueError): ev.stop_pose_evidence(data, end, hold)


def test_runtime_stop_break_and_verdict_both_consume_evidence():
    source = (Path(__file__).resolve().parents[1]/'scripts/validate_g1_traffic.py').read_text(encoding='utf-8')
    assert source.count('ev.stop_pose_evidence(') == 2
    assert "if evidence['passed']:" in source
    assert "and evidence['passed'])" in source
    assert "'pose_evidence': evidence" in source
    assert "'dynamics_source_s'" in source and "'odom_source_s'" in source


def test_evaluation_changes_have_explicit_successor_not_rewritten_old_contract():
    root = Path(__file__).resolve().parents[1]
    contract = json.loads((root/'config/tests/dense_stop_evidence_r0_sha256.json').read_text())
    assert hashlib.sha256((root/contract['parent_contract']).read_bytes()).hexdigest() == contract['parent_contract_sha256']
    successors, parent = [], 'config/tests/dense_stop_evidence_r0_sha256.json'
    for next_name in ('lidar_stress_eval_r0_sha256.json', 'per_ray_occupancy_r0_sha256.json', 'corner_anchor_r0_sha256.json', 'corner_distance_r0_sha256.json', 'corridor_fallback_r0_sha256.json', 'recent_observation_r0_sha256.json'):
        successor = json.loads((root/'config/tests'/next_name).read_text())
        assert successor['parent_contract'] == parent
        assert hashlib.sha256((root/parent).read_bytes()).hexdigest() == successor['parent_contract_sha256']
        successors.append(successor)
        parent = 'config/tests/'+next_name
    for name, entry in contract['approved_evaluation_scripts'].items():
        for successor in successors:
            later = successor['approved_evaluation_scripts'].get(name)
            if later:
                assert later['before'] == entry['after']
                entry = later
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == entry['after']
