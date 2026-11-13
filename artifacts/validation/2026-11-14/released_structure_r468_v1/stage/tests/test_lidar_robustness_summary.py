import importlib.util
from pathlib import Path

SPEC = importlib.util.spec_from_file_location('robustness_summary',
    Path(__file__).resolve().parents[1]/'scripts/summarize_lidar_robustness.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def evidence():
    report = {'requested_lidar_impairment': {'delay_s': .3}, 'passed': False,
              'lidar_impairment_audit': {'verified': True}, 'sequential_acquisition_verified': True,
              'shutdown_clean': True, 'remaining_test_pids': []}
    rows = [{'time': t, 'autonomy': {'started': True}, 'safe_drive_speed_mps': 0.,
             'dynamics': {'truth_longitudinal_mps': 0., 'truth_lateral_mps': 0.},
             'safety': {'reason': 'stale_input'}, 'collision': False} for t in range(8)]
    return report, rows


def test_expected_stale_protection_is_separate_from_lap_success():
    report, rows = evidence()
    assert MODULE.audit_expected_stale_rejection(report, rows)['protection_passed']
    assert report['passed'] is False
    assert MODULE.audit_expected_stale_rejection(report, [])['protection_passed'] is False
    assert MODULE.audit_expected_stale_rejection({}, rows) is None


def test_motion_or_positive_command_or_unclean_exit_invalidates_protection():
    for fault in ('command', 'motion', 'shutdown', 'collision', 'reason'):
        report, rows = evidence()
        if fault == 'command': rows[2]['safe_drive_speed_mps'] = .1
        if fault == 'motion': rows[2]['dynamics']['truth_longitudinal_mps'] = .1
        if fault == 'shutdown': report['shutdown_clean'] = False
        if fault == 'collision': rows[2]['collision'] = True
        if fault == 'reason': rows[2]['safety']['reason'] = 'clear'
        assert not MODULE.audit_expected_stale_rejection(report, rows)['protection_passed']
