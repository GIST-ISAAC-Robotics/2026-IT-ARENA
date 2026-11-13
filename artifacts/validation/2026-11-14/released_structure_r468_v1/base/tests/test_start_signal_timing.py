import pytest

from validate_start_signal_timing import Scenario, TimedWorker, run_case, stage


def test_nominal_timeline_boundaries():
    case = Scenario('max', red3_s=3., red4_s=3.)
    assert case.green_s == 9.
    assert [stage(t, case) for t in (0, 1, 2, 3, 6, 9, 14)] == [
        (0, False), (1, False), (2, False), (3, False), (4, False), (0, True), (0, False)]


def test_work_is_not_delivered_early_or_twice():
    worker = TimedWorker(.15)
    worker.result, worker.due = {'test': True}, .15
    assert worker.poll() is None
    worker.now = .15
    assert worker.poll() == {'test': True}
    assert worker.poll() is None


@pytest.mark.parametrize('fault,expected', [('', 'start'), ('duplicates', 'start'),
                                         ('drop_transition', 'no_start'), ('reset_on_green', 'no_start')])
def test_time_scenarios_apply_real_pipeline_state(fault, expected):
    row = run_case(Scenario(fault or 'nominal', expected=expected, fault=fault, camera_hz=10.))
    assert row['passed'], row
    assert not row['false_start']
    if expected == 'start':
        assert row['decision_delay_s'] >= .2
        assert row['off_keeps_start']
    if fault == 'duplicates':
        assert row['stats']['out_of_order'] > 0
    if fault == 'reset_on_green':
        assert row['stats']['resets'] == 1


def test_watchdog_is_an_explicit_fault_not_quiet_wait():
    row = run_case(Scenario('watchdog', expected='no_start', work_delay_s=1.1, camera_hz=10.))
    assert row['passed']
    assert 'watchdog' in row['fault']
