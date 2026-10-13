import sys
import copy
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_solver_compare import actual_solver, world_without_solver
import analyze_solver_compare as analysis


def test_getter_not_sdf_or_component():
    assert not actual_solver('requested solver pgs, PhysicsSolver=pgs', 'pgs')['passed']
    assert actual_solver('\x1b[1;32mUsing [PgsBoxedLcpSolver] solver.\x1b[0m', 'pgs')['passed']
    assert not actual_solver('Using [DantzigBoxedLcpSolver] solver.', 'pgs')['passed']
    assert not actual_solver('Using [PgsBoxedLcpSolver] solver.\nUsing [DantzigBoxedLcpSolver] solver.', 'pgs')['passed']
    assert not actual_solver('Failed to cast constraint solver\nUsing [PgsBoxedLcpSolver] solver.', 'pgs')['passed']


def test_only_solver_difference_normalized(tmp_path):
    a, b = tmp_path/'a.sdf', tmp_path/'b.sdf'
    def write(path, solver='', mu='0.9'):
        path.write_text(f'<sdf><world><physics><max_step_size>0.001</max_step_size><dart>'
            f'<collision_detector>ode</collision_detector>{solver}</dart></physics><model><mu>{mu}</mu></model></world></sdf>')
    tag = '<solver><solver_type>pgs</solver_type></solver>'
    write(a)
    write(b, tag)
    assert world_without_solver(a) == world_without_solver(b, 'pgs')
    write(b, tag, '1.0')
    assert world_without_solver(a) != world_without_solver(b, 'pgs')
    with pytest.raises(ValueError):
        world_without_solver(b, 'dantzig')
    with pytest.raises(ValueError):
        world_without_solver(b)
    write(b, tag+tag)
    with pytest.raises(ValueError):
        world_without_solver(b, 'pgs')


def test_missing_contact_is_unknown_not_zero_or_success(tmp_path, monkeypatch):
    row = dict(sim_time_s=1., truth_roll_rad=0., truth_pitch_rad=0., truth_z_m=.1,
               truth_longitudinal_mps=.8, wheel_surface_speed_mps=0.)
    monkeypatch.setattr(analysis, 'rows', lambda _: [row])
    monkeypatch.setattr(analysis, 'model_params', lambda _: {})
    monkeypatch.setattr(analysis, 'read_contacts', lambda _: ({'rear_left_wheel': {}, 'rear_right_wheel': {}}, 0))
    def forbidden(*args):
        raise AssertionError('incomplete contact data must not enter impulse/force audit')
    monkeypatch.setattr(analysis, 'exact_sample', forbidden)
    monkeypatch.setattr(analysis, 'physics_audit', forbidden)
    result = analysis.force_audit(tmp_path, 1.)
    assert result['point_bound_ratio_max'] is None
    assert result['planar_audit'] is None
    assert result['exact_contact_coverage_complete'] is False
    assert len(result['missing_exact_contacts']) == 1


def test_input_integrity_does_not_claim_stop_or_ignore_other_failure():
    summary = {'passed': False, 'checks': {'evidence': False, 'posthoc_trace_run': False, 'ack': True},
        'evidence': {'checks': {'stopped': False, 'raw': True}, 'failed_schedule_checks': {'a': []}},
        'trace': {'validity': {'stopped': False, 'report_passed': False, 'passed': False, 'input_valid': True}}}
    result = analysis.input_integrity(summary)
    assert result['passed'] and not result['stopped'] and not result['physical_run_passed']
    for section, key in (('evidence', 'raw'), ('trace', 'input_valid')):
        bad = copy.deepcopy(summary)
        bad[section]['checks' if section == 'evidence' else 'validity'][key] = False
        assert not analysis.input_integrity(bad)['passed']
    bad = copy.deepcopy(summary)
    bad['evidence']['failed_schedule_checks']['a'] = ['lateness']
    assert not analysis.input_integrity(bad)['passed']
