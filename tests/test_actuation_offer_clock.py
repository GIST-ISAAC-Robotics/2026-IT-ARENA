"""오프라인 offer 시각 후보. 원래 임대 나이/불확실성 보존, 구동 허가 금지."""
from dataclasses import replace
from fractions import Fraction
import json
from pathlib import Path
import subprocess
import sys

import pytest

from audit_actuation_offer_clock import assess_offer, cases, fixture
from arena_vehicle_interface.clock_bounds import ClockAssumptions, ClockExchange, fit_clock
from arena_vehicle_interface.clock_freshness import HostTimeWindow


def review(data):
    return assess_offer(*data[:4], **data[4])


@pytest.mark.parametrize('name,data,expected', list(cases()), ids=lambda x: x if isinstance(x, str) else None)
def test_named_evidence_cases(name, data, expected):
    if expected is None:
        with pytest.raises(ValueError):
            review(data)
    else:
        result = review(data)
        assert result['timestamp_policy_satisfied'] is expected, (name, result)
        assert result['ready_to_drive'] is False and result['runtime_connected'] is False
        assert result['source_timestamp_checked'] is False and result['hardware_assumptions_verified'] is False


def test_receipt_does_not_restart_expired_lease():
    data = fixture(now_ns=9_120_000_000, received_ns=9_119_000_000)
    result = review(data)
    # 잘못된 '받은 뒤 100 ms' 식은 true. 후보는 원래 MCU 만료를 유지해 false.
    assert data[3].upper_ns-data[2].lower_ns < 100_000_000
    assert result['timestamp_policy_satisfied'] is False
    assert result['expiry_margin_lower_ns'] < 0
    assert result['offer']['lease_issued_us'] == 1_000_100


def test_midpoint_deadline_would_accept_but_earliest_expiry_does_not():
    data = fixture(now_ns=9_098_000_000, delay_ns=5_000_000)
    result = review(data)
    expiry = result['mapped_expiry']
    midpoint = (expiry['host_lower_ns']+expiry['host_upper_ns'])//2
    assert result['arrival_upper_ns'] < midpoint
    assert result['timestamp_policy_satisfied'] is False


@pytest.mark.parametrize('delta,expected', [(-1, True), (0, False), (1, False)])
def test_strict_expiry_endpoint(delta, expected):
    data = list(fixture())
    earliest = data[1].map_interval(data[0]['lease_expires_us'], boot='boot-A').host_lower_ns
    now = earliest-data[4]['forward_delay_upper_ns']+delta
    data[3] = HostTimeWindow('host-epoch-A', now, now)
    assert review(data)['timestamp_policy_satisfied'] is expected


@pytest.mark.parametrize('field,value', [('v', True), ('v', 2), ('lease', 0), ('lease', True),
                                      ('generation', -1), ('lease_issued_us', 1.),
                                      ('lease_expires_us', 1_000_100), ('lease_expires_us', 1_100_101),
                                      ('owner', []), ('output_ready', 1), ('state', []),
                                      ('state', 'UNKNOWN'), ('boot', ''), ('boot', 'b'*65)])
def test_offer_shape_and_limits_rejected(field, value):
    data = list(fixture())
    data[0][field] = value
    with pytest.raises(ValueError):
        review(data)


@pytest.mark.parametrize('field,value', [('forward_delay_upper_ns', None), ('forward_delay_upper_ns', True),
                                      ('forward_delay_upper_ns', -1), ('forward_delay_upper_ns', 2**63-1),
                                      ('max_lease_us', 0), ('max_issue_age_ns', 0), ('policy_note', ''),
                                      ('expected_generation', True), ('mapping_host_epoch', 'different')])
def test_explicit_assumptions_not_silent_defaults(field, value):
    data = list(fixture())
    data[4][field] = value
    with pytest.raises(ValueError):
        review(data)


@pytest.mark.parametrize('which', ['receipt_future', 'receipt_before_issue', 'receipt_overlaps_issue',
                                 'decision_future_overlap', 'decision_wide'])
def test_uncertain_or_reversed_order_not_approved(which):
    data = list(fixture())
    if which == 'receipt_future': data[2] = HostTimeWindow('host-epoch-A', 9_030_000_000, 9_030_000_000)
    if which == 'receipt_before_issue': data[2] = HostTimeWindow('host-epoch-A', 8_990_000_000, 8_990_000_000)
    if which == 'receipt_overlaps_issue': data[2] = HostTimeWindow('host-epoch-A', 9_000_000_000, 9_010_000_000)
    if which == 'decision_future_overlap': data[3] = HostTimeWindow('host-epoch-A', 9_005_000_000, 9_020_000_000)
    if which == 'decision_wide': data[3] = HostTimeWindow('host-epoch-A', 9_020_000_000, 9_100_000_000)
    assert review(data)['timestamp_policy_satisfied'] is False


def test_exact_large_host_clock_offset_does_not_change_decision():
    original = list(fixture())
    shift = 2**54+7
    data = list(fixture())
    data[1] = replace(data[1], host_anchor_ns=data[1].host_anchor_ns+shift)
    data[2] = replace(data[2], lower_ns=data[2].lower_ns+shift, upper_ns=data[2].upper_ns+shift)
    data[3] = replace(data[3], lower_ns=data[3].lower_ns+shift, upper_ns=data[3].upper_ns+shift)
    first, second = review(original), review(data)
    assert first['conditions'] == second['conditions']
    assert first['expiry_margin_lower_ns'] == second['expiry_margin_lower_ns']
    assert first['age']['age_upper_ns'] == second['age']['age_upper_ns']


@pytest.mark.parametrize('ppm', [-200, 0, 200])
def test_accepted_sample_precedes_true_expiry_under_declared_affine_clock(ppm):
    rate = Fraction(1_000_000+ppm, 1_000_000)
    offset = 8_000_000_000
    def truth(mcu): return int(rate*mcu*1000+offset)
    assumptions = ClockAssumptions(200, 0, 0, 5_000_000, 200_000)
    exchange = ClockExchange('boot-A', 1, truth(1_000_000)-1_000_000,
                             1_000_000, 1_000_100, truth(1_000_100)+9_000_000)
    fit = fit_clock([exchange], assumptions)
    accepted, rejected = 0, 0
    for age_ms in range(10, 151, 5):
        data = list(fixture(now_ns=truth(1_000_100)+age_ms*1_000_000,
                            received_ns=truth(1_000_100)+10_000_000))
        data[1] = fit
        result = review(data)
        if result['timestamp_policy_satisfied']:
            accepted += 1
            assert result['arrival_upper_ns'] < truth(1_100_100)
            assert 0 <= data[3].upper_ns-truth(1_000_100) < 100_000_000
        else:
            rejected += 1
    assert accepted > 0 and rejected > 0


def test_stopped_or_not_output_ready_is_never_promoted_to_drive_permission():
    for state in ('DISARMED', 'ESTOP_LATCHED', 'FAULT_LATCHED'):
        data = list(fixture())
        data[0].update(state=state, owner=None, output_ready=False)
        result = review(data)
        assert result['timestamp_policy_satisfied'] is True  # 시간 조건만 통과
        assert result['ready_to_drive'] is False and result['session_evidence_verified'] is False


def test_cli_outputs_new_evidence_and_will_not_overwrite(tmp_path):
    root = Path(__file__).resolve().parents[1]
    output = tmp_path/'new'
    command = [sys.executable, '-B', str(root/'scripts/audit_actuation_offer_clock.py'), '--output', str(output)]
    first = subprocess.run(command, capture_output=True, text=True, timeout=30)
    assert first.returncode == 0, first.stderr
    saved = (output/'report.json').read_bytes()
    report = json.loads(saved)
    assert report['passed'] and report['source_unchanged'] and len(report['cases']) == 10
    assert report['ready_to_drive'] is False and report['runtime_connected'] is False
    assert subprocess.run(command, capture_output=True, text=True, timeout=30).returncode != 0
    assert (output/'report.json').read_bytes() == saved


def test_candidate_not_imported_by_production_nodes():
    import ast
    root = Path(__file__).resolve().parents[1]
    for package in ('arena_autonomy', 'arena_vehicle_interface'):
        for path in (root/'src'/package/package).glob('*.py'):
            names = [node.module for node in ast.walk(ast.parse(path.read_text(encoding='utf-8')))
                     if isinstance(node, ast.ImportFrom)]
            assert 'audit_actuation_offer_clock' not in names
