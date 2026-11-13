"""G1 C1 스트레스 주입 자체의 증거 검사."""
from dataclasses import asdict
import math

import pytest

from arena_vehicle_interface.lidar_impairment import ImpairmentConfig, ScanImpairment
from g1_lidar_stress import audit_injection, load_test_profile, require_injection_evidence


def fixture(delay=.05, jitter=.02, amplitude=.03):
    cfg = asdict(ImpairmentConfig(mode='uniform', amplitude_m=amplitude, delay_s=delay, jitter_s=jitter))
    records, scans = [], []
    for i in range(3):
        first = 1+i*.1
        complete = first+.1
        records.append(dict(first_ray_stamp_s=first, last_ray_stamp_s=first+.0998,
            completion_stamp_s=complete, requested_release_stamp_s=complete+delay,
            release_source_stamp_s=complete+delay+.001, injected_delay_s=delay+.001,
            max_capture_time_error_s=.001, model='sequential', discarded_source_gaps=0,
            impairment=dict(config=cfg.copy(), samples=500*(i+1), scans=i+1, invalidated=0,
                            mean_error_m=.0001 if amplitude else 0, rms_error_m=.016 if amplitude else 0,
                            max_abs_error_m=.029 if amplitude else 0)))
        scans.append(dict(stamp_s=first, time_increment=.0002, ranges=[1.]*500))
    return records, scans, cfg


def test_config_and_actual_source_header_release_and_noise_match():
    result = audit_injection(*fixture())
    assert result['verified'] and result['received_scans'] == 3
    assert result['minimum_added_delay_s'] == pytest.approx(.051)


@pytest.mark.parametrize('fault', ['absent', 'clean', 'wrong_config', 'delay_hidden', 'future_point',
    'no_noise', 'too_much_noise', 'nan', 'source_gap', 'time_rewritten', 'empty_received',
    'bad_increment', 'duplicate', 'long_queue', 'bad_count', 'missing_metadata'])
def test_injection_contract_rejects_missing_or_wrong_evidence(fault):
    rows, scans, cfg = fixture()
    row = rows[1]
    if fault == 'absent': rows = []
    if fault == 'clean': del row['impairment']
    if fault == 'wrong_config': row['impairment']['config']['seed'] = 99
    if fault == 'delay_hidden': row['injected_delay_s'] = 0
    if fault == 'future_point': row['last_ray_stamp_s'] = 10
    if fault == 'no_noise': rows[-1]['impairment']['max_abs_error_m'] = 0
    if fault == 'too_much_noise': row['impairment']['max_abs_error_m'] = .031
    if fault == 'nan': row['completion_stamp_s'] = math.nan
    if fault == 'source_gap': row['discarded_source_gaps'] = 1
    if fault == 'time_rewritten': scans[1]['stamp_s'] += .05
    if fault == 'empty_received': scans = []
    if fault == 'bad_increment': scans[1]['time_increment'] = 0
    if fault == 'duplicate': rows[1] = rows[0].copy()
    if fault == 'long_queue': row['requested_release_stamp_s'] -= .04
    if fault == 'bad_count': row['impairment']['samples'] = True
    if fault == 'missing_metadata': del row['completion_stamp_s']
    assert not audit_injection(rows, scans, cfg)['verified']


def test_delay_only_does_not_require_noise_and_missing_scans_not_zero():
    assert audit_injection(*fixture(amplitude=0))['verified']
    rows, scans, cfg = fixture()
    assert not audit_injection(rows, scans[1:]+[scans[-1]], cfg)['verified']


def test_gaussian_needs_separate_registration():
    rows, scans, cfg = fixture()
    cfg['mode'] = 'gaussian'
    with pytest.raises(ValueError): audit_injection(rows, scans, cfg)


def test_fixed_positive_bias_requires_observed_sign_and_magnitude():
    rows, scans, cfg = fixture(amplitude=0)
    cfg.update(mode='none', bias_m=.03)
    for row in rows:
        row['impairment'].update(config=cfg.copy(), mean_error_m=.03, rms_error_m=.03, max_abs_error_m=.03)
    assert audit_injection(rows, scans, cfg)['verified']
    rows[1]['impairment']['mean_error_m'] = -.03
    assert not audit_injection(rows, scans, cfg)['verified']


def test_profile_is_optional_repository_scoped_and_validated(tmp_path):
    assert load_test_profile(None, tmp_path) == (None, None)
    profile = tmp_path/'profile.json'
    profile.write_text('{"mode":"uniform","amplitude_m":0.03}')
    path, config = load_test_profile(profile, tmp_path)
    assert path == profile.resolve() and config['amplitude_m'] == .03
    with pytest.raises(ValueError): load_test_profile(profile, tmp_path/'nested')
    for invalid in ('{"mode":"gaussian"}', '{"delay_s":-1}', '{"unexpected":1}'):
        profile.write_text(invalid)
        with pytest.raises((ValueError, TypeError)): load_test_profile(profile, tmp_path)


@pytest.mark.parametrize('prior,verified,controller,safety,expected', [
    (True, True, True, True, True), (False, True, True, True, False),
    (True, False, True, True, False), (True, True, False, True, False),
    (True, True, True, False, False)])
def test_additional_injection_gate_never_overrides_previous_failure(prior, verified, controller, safety, expected):
    report = dict(passed=prior, run_valid=prior, integrity={'old_check': prior},
                  autonomy_subscriptions=['/scan'] if controller else [],
                  safety_subscriptions=['/scan'] if safety else [])
    result = require_injection_evidence(report, dict(verified=verified))
    assert result['passed'] is expected and result['run_valid'] is expected
    assert result['integrity']['old_check'] is prior


@pytest.mark.parametrize('settings', [dict(mode='uniform', amplitude_m=.03),
    dict(bias_m=.03), dict(mode='uniform', amplitude_m=.03, delay_s=.05, jitter_s=.02, seed=23)])
def test_real_impairment_queue_produces_auditable_evidence(settings):
    # 실제 어댑터와 같은 주입/큐 구현을 사용하되 ROS/Gazebo 주행 증거로 부르지 않는다.
    config = ImpairmentConfig(**settings)
    model = ScanImpairment(config)
    records, received = [], []
    for tick in range(600):
        stamp = .002+tick*.002
        ready = model.advance(stamp)
        if tick in range(50, 501, 50):
            first = stamp-.1
            scan = dict(stamp_s=first, time_increment=.0002,
                        ranges=model.distort([1.]*500, .05, 12.))
            model.enqueue(scan, stamp, dict(first_ray_stamp_s=first, last_ray_stamp_s=first+.0998,
                max_capture_time_error_s=.0018, discarded_source_gaps=0, model='sequential'))
            ready += model.advance(stamp)
        for scan, record in ready:
            records.append(record)
            received.append(scan)
    result = audit_injection(records, received, asdict(config))
    assert result['verified'], result
    assert result['received_scans'] == 10


def test_evaluator_snapshots_selected_profile_and_implementation():
    import validate_g1_traffic as harness
    profile = harness.REPO/'config/tests/lidar_uniform30.json'
    paths = harness.source_paths(harness.CONFIG, profile)
    assert paths.count('config/tests/lidar_uniform30.json') == 1
    assert 'scripts/g1_lidar_stress.py' in paths
    assert 'src/arena_vehicle_interface/arena_vehicle_interface/lidar_impairment.py' in paths
    assert 'config/tests/lidar_uniform30.json' not in harness.source_paths(harness.CONFIG)
    with pytest.raises(ValueError): harness.source_paths(harness.CONFIG, harness.REPO.parent/'outside.json')
