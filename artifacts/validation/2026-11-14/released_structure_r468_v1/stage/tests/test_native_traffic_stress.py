import importlib.util
from pathlib import Path
import sys

import pytest

from g1_lidar_stress import audit_injection, require_injection_evidence
from test_native_traffic_contract import fixture, PROFILE
from test_g1_lidar_stress import fixture as uniform_fixture


def noisy():
    _, records, received = fixture()
    _, _, config = uniform_fixture()
    for i, record in enumerate(records):
        first = record['first_ray_stamp_s']
        record.update(completion_stamp_s=first+.1, requested_release_stamp_s=first+.15,
            release_source_stamp_s=first+.151, injected_delay_s=.051,
            impairment=dict(config=config.copy(), samples=500*(i+1), scans=i+1, invalidated=0,
                mean_error_m=.0001, rms_error_m=.016, max_abs_error_m=.029))
    return records, received, config


def test_native_geometry_and_same_noise_delay_contract():
    out = audit_injection(*noisy(), native_profile=PROFILE)
    assert out['verified'] and out['native_sampling']['verified']
    assert out['native_sampling']['sample_counts'] == [504, 508, 512]
    assert out['maximum_added_delay_s'] == pytest.approx(.051)
    assert len(out['checks']) == 6


@pytest.mark.parametrize('fault', ['delay', 'noise', 'angle', 'epoch', 'unknown', 'fixed500'])
def test_native_selection_does_not_weaken_existing_evidence(fault):
    records, received, config = noisy()
    if fault == 'delay': records[1]['injected_delay_s'] = 0
    if fault == 'noise': records[1]['impairment']['max_abs_error_m'] = .04
    if fault == 'angle': received[1]['angles_rad'][50] += .01
    if fault == 'epoch': received[1]['source_epoch'] = True
    if fault == 'unknown': received[1]['ranges'].pop()
    if fault == 'fixed500': received[1]['ranges'] = [1.]*500
    assert not audit_injection(records, received, config, native_profile=PROFILE)['verified']


@pytest.mark.parametrize('selected,observed,expected', [('/scan','/scan',True),
    ('/scan/native','/scan/native',True), ('/scan','/scan/native',False), ('/scan/native','/scan',False)])
def test_exact_selected_input_is_required(selected, observed, expected):
    report = dict(passed=True, run_valid=True, integrity={},
        autonomy_subscriptions=[observed], safety_subscriptions=[observed])
    require_injection_evidence(report, {'verified':True}, lidar_topic=selected)
    assert report['passed'] is expected


def test_uniform_evidence_values_unchanged_except_descriptive_limit():
    root = Path(__file__).resolve().parents[1]
    import types, zipfile
    old = types.ModuleType('old_g1_stress')
    archive = root/'artifacts/validation/2026-11-11/native_traffic_r79_v1/before_r79.zip'
    with zipfile.ZipFile(archive) as bundle:
        code = bundle.read('scripts/g1_lidar_stress.py')
    exec(compile(code, '<r78-g1-lidar-stress>', 'exec'), old.__dict__)
    a, b = old.audit_injection(*uniform_fixture()), audit_injection(*uniform_fixture())
    a.pop('limits');b.pop('limits')
    assert a == b and b['verified']
