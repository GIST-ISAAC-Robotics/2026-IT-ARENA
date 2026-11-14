import copy

import pytest

from m4_operation_trial import TrialError
from m4_second_start import SecondStartTrial, evaluate_second_start
from test_m4_operation_trial import full_sequence, new_race_evidence


def fixture():
    original, race, mcu = full_sequence(True)
    return SecondStartTrial(), original.snapshot(), race, mcu


def light(now, color):
    return {'sim_time_s': now, 'applied': color, 'error': ''}


def sequence():
    signal, op, race, mcu = fixture()
    assert signal.tick(7.7, op, race, mcu, light(7.7, 'green')) is None
    assert signal.tick(9.21, op, race, mcu, light(9.21, 'green'))['color'] == 'red'
    signal.complete({'success': True}, 9.22)
    # 수락 응답만으로 빨강 적용을 주장하지 않는다.
    assert signal.tick(9.23, op, race, mcu, light(9.23, 'green')) is None
    assert signal.red_s is None
    signal.tick(9.3, op, race, mcu, light(9.3, 'red'))
    assert signal.tick(11.31, op, race, mcu, light(11.31, 'red'))['color'] == 'green'
    signal.complete({'success': True}, 11.32)
    signal.tick(11.4, op, race, mcu, light(11.4, 'green'))
    assert signal.phase == 'wait_pixels'
    race.update(started=True, visual_start_fact=True, policy_allowed_if_sensors_ready=True)
    signal.tick(11.5, op, race, mcu, light(11.5, 'green'))
    assert signal.phase == 'done'
    return signal


def test_second_signal_uses_service_applied_state_and_new_pixels_separately():
    s = sequence()
    assert s.red_s == 9.3 and s.green_s == 11.4 and s.done_s == 11.5
    assert [e['color'] for e in s.events if e['event'] == 'request'] == ['red', 'green']


@pytest.mark.parametrize('change', ['old_fact', 'boot', 'epoch', 'new_stop', 'disarmed'])
def test_second_signal_cannot_cover_permission_or_identity_failure(change):
    s, op, r, m = fixture()
    s.tick(7.7, op, r, m, light(7.7, 'green'))
    if change == 'old_fact': r['visual_start_fact'] = True
    elif change == 'boot': m['boot'] = 'new'
    elif change == 'epoch': r['vision_epoch'] += 1
    elif change == 'new_stop': r['intervention'] += 1
    elif change == 'disarmed': m['state'] = 'DISARMED'
    with pytest.raises(TrialError): s.tick(8., op, r, m, light(8., 'green'))


def test_signal_lost_response_is_failure_not_retry():
    s, op, r, m = fixture()
    s.tick(7.7, op, r, m, light(7.7, 'green'))
    s.tick(9.21, op, r, m, light(9.21, 'green'))
    assert s.tick(9.5, op, r, m, light(9.5, 'red')) is None
    with pytest.raises(TrialError, match='timeout'): s.tick(11.22, op, r, m, light(11.22, 'red'))
    assert len([e for e in s.events if e['event'] == 'request']) == 1


def test_old_or_future_application_stamp_is_not_rendering_evidence():
    s, op, r, m = fixture()
    s.tick(7.7, op, r, m, light(7.7, 'green'))
    s.tick(9.21, op, r, m, light(9.21, 'green')); s.complete({'success': True}, 9.22)
    for stamp in (8., 9.2, 10.):
        s.tick(9.23, op, r, m, light(stamp, 'red'))
        assert s.red_s is None


def evidence():
    data, report, ego, mcu = new_race_evidence()
    signal = sequence().snapshot()
    data['second_start'] = signal
    for s in data['samples']:
        if s['time_s'] >= 11.5:
            s['race'].update(started=True, visual_start_fact=True, policy_allowed_if_sensors_ready=True,
                             policy_reason='race_permitted')
            s['safe_speed'] = .35
    ego = [(t, x+.25*max(0., t-11.6), y, a) for t, x, y, a in ego]
    return data, report, ego, mcu


def test_second_start_requires_actual_new_motion_and_preserves_old_traffic_failure():
    data, report, ego, mcu = evidence()
    before = copy.deepcopy((data, report, ego, mcu))
    result = evaluate_second_start(data, report, ego, mcu)
    assert result['passed'], result
    assert (data, report, ego, mcu) == before
    assert report['gates']['restart'] is False


@pytest.mark.parametrize('defect', ['no_move', 'early_move', 'old_fact', 'no_new_pixels', 'pose_gap',
                                  'status_gap', 'wrong_epoch', 'red_short', 'double_green', 'service_refused',
                                  'mismatched_event_time', 'out_of_order_response'])
def test_second_start_missing_or_contradictory_evidence_fails(defect):
    data, report, ego, mcu = evidence()
    s = data['second_start']
    if defect == 'no_move': ego = [(t, min(x, 1.1), y, a) for t, x, y, a in ego]
    elif defect == 'early_move': ego = [(t, x+.03*max(0., t-9.), y, a) for t, x, y, a in ego]
    elif defect == 'old_fact': data['samples'][420]['race']['visual_start_fact'] = True
    elif defect == 'no_new_pixels': s['events'] = [e for e in s['events'] if e['event'] != 'visual_start']
    elif defect == 'pose_gap': ego = [p for p in ego if not 12.<p[0]<12.2]
    elif defect == 'status_gap': data['samples'] = [r for r in data['samples'] if not 12.<r['time_s']<12.2]
    elif defect == 'wrong_epoch': data['samples'][-1]['race']['vision_epoch'] += 1
    elif defect == 'red_short': s['red_s'] = s['green_request_s']-1.
    elif defect == 'double_green': s['events'].append({'event': 'request', 'color': 'green', 'time_s': 12.})
    elif defect == 'service_refused': next(e for e in s['events'] if e['event'] == 'response')['response']['success'] = False
    elif defect == 'mismatched_event_time': s['green_s'] += .1
    elif defect == 'out_of_order_response': next(e for e in s['events'] if e['event'] == 'response')['time_s'] = 1.
    assert not evaluate_second_start(data, report, ego, mcu)['passed']
