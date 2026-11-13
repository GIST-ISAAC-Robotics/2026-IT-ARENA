"""운영 어댑터 표시·요청 바인딩·기록 내보내기. ROS 없이 순수 상태만 확인한다."""
import json

from arena_autonomy import race_ops_export
from arena_autonomy.race_ops_core import OpsCore, display_state


def status(**race):
    base = dict(session='s', race_id='r', race_epoch=0, intervention=2, latch='explicit_stop', started=True,
                recovery_ready=False, resume=None)
    return dict(state='RACE_LATCHED', side='right', sim_time_s=10., race={**base, **race}, reason=None)


def core_with(**race):
    core = OpsCore('car1', status_stale_s=1.)
    core.update_status(status(**race), 100.)
    return core


VIEW = dict(session='s', race_id='r', race_epoch=0, intervention=2)


def test_display_distinguishes_operating_states():
    assert display_state(None, None) == 'unknown'
    assert display_state(status(), {}) == 'operator_stop'
    assert display_state(status(recovery_ready=True), {}) == 'resumable'
    assert display_state(status(resume=dict(phase='inputs')), {}) == 'recovery'
    assert display_state(status(latch='process_start', started=False), {}) == 'setup'
    assert display_state(status(latch=None, started=False), {}) == 'signal'
    assert display_state({**status(latch=None), 'state': 'RUNNING'}, {}) == 'running'
    assert display_state({**status(latch=None), 'state': 'FOLLOW_GAP_HOLD'}, {}) == 'yield_hold'
    assert display_state({**status(latch=None), 'state': 'SENSOR_STOP'}, {}) == 'fault'
    assert display_state(status(latch='fault:vision'), {}) == 'fault'


def test_old_view_is_refused_without_manual_numbers():
    core = core_with(recovery_ready=True)
    old = {**VIEW, 'intervention': 1}
    entry = core.submit('resume_start', 'a'*8, old, 'c', None, 100.1)
    assert entry['stage'] == 'rejected_by_adapter' and entry['adapter_reason'] == 'stale_view'
    assert core.next_action() is None
    entry = core.submit('resume_start', 'b'*8, VIEW, 'c', None, 100.1)
    assert entry['stage'] == 'queued' and core.next_action()['request'] == 'b'*8


def test_duplicate_request_is_not_executed_twice():
    core = core_with(recovery_ready=True)
    core.submit('resume_start', 'same-id', VIEW, 'c', None, 100.1)
    again = core.submit('resume_start', 'same-id', VIEW, 'c', None, 100.2)
    assert again['request'] == 'same-id' and core.next_action() is not None and core.next_action() is None


def test_stop_needs_no_view_even_when_status_is_stale():
    core = core_with()
    entry = core.submit('stop', 'stop-1', None, 'c', None, 200.)
    assert entry['stage'] == 'queued' and entry['physical_stop'] == '미확인'
    start = core.submit('resume_start', 'late-1', VIEW, 'c', None, 200.)
    assert start['adapter_reason'] == 'status_stale'


def test_new_race_and_restore_need_explicit_confirmation():
    core = core_with()
    assert core.submit('new_race', 'n1', VIEW, 'c', None, 100.1)['adapter_reason'] == 'new_race_confirmation_required'
    assert core.submit('new_race', 'n2', VIEW, 'c', 'NEW_RACE', 100.1)['stage'] == 'queued'


def test_unanswered_command_is_unknown_not_failed():
    core = core_with(recovery_ready=True)
    entry = core.commands[core.submit('resume_start', 'x1', VIEW, 'c', None, 100.1)['request']]
    core.mark(entry, 100.2, stage='sent', sent_wall=100.2)
    core.expire(102., 3.)
    assert entry['stage'] == 'sent'
    core.expire(103.3, 3.)
    assert entry['stage'] == 'unknown_after_send'


def test_restore_offer_only_for_fresh_process_and_other_session():
    record = dict(race_id='old', session='old-s', side='right', vehicle_id='car1', saved_wall_time=1000.)
    core = OpsCore('car1')
    core.set_record(record)
    core.update_status(status(session='new-s', latch='process_start', started=False), 100.)
    assert core.restore_offer(1100.)['race_id'] == 'old'
    assert core.restore_offer(1000.+7201.) is None  # 임의의 오래된 기록을 쓰지 않는다.
    assert core.view(100.1, 1100.)['controls']['restore_race']['allowed']
    view = dict(session='new-s', race_id='r', race_epoch=0, intervention=2)
    assert core.submit('restore_race', 'r1', view, 'c', None, 100.1)['adapter_reason'] == 'same_race_confirmation_required'
    core.update_status(status(session='new-s', latch='process_start', started=True), 100.2)
    assert core.restore_offer(1100.) is None
    other = OpsCore('car2')
    other.set_record(record)
    other.update_status(status(session='new-s', latch='process_start', started=False), 100.)
    assert other.restore_offer(1100.) is None


def test_record_holds_only_minimal_logical_facts():
    core = core_with()
    payload = core.record_payload(5.)
    assert set(payload) == {'v', 'vehicle_id', 'race_id', 'session', 'race_epoch', 'start_source', 'side',
                            'side_marker', 'saved_wall_time'}
    assert core_with(started=False).record_payload(5.) is None


def test_view_labels_speed_sources_and_never_claims_physical_stop():
    core = core_with()
    core.update_drive('/drive', 1.2, .1, 100.)
    core.update_drive('/drive/safe', .8, .1, 100.)
    core.update_mcu(dict(state='ACTIVE', reason='drive', target_speed_mps=.8, mean_wheel_mps=.7, feedback_valid=True), 100.)
    view = core.view(100.1, 5.)
    assert view['speeds']['plan_mps'] == 1.2 and view['speeds']['protected_mps'] == .8
    assert view['speeds']['mcu_target_mps'] == .8 and view['speeds']['motor_shaft_equiv_mps'] == .7
    assert view['limits']['physical_stop'] == '미확인'
    json.dumps(view, allow_nan=False)


def test_stale_flag_and_disabled_controls():
    core = core_with(recovery_ready=True)
    assert core.view(100.5, 5.)['controls']['resume_start']['allowed']
    view = core.view(102., 5.)
    assert view['connection']['status_stale'] and not view['controls']['resume_start']['allowed']
    assert view['controls']['stop']['allowed']


def test_ordinary_stop_start_uses_supervisor_readiness_without_faking_reset():
    core = core_with(resume_start_ready=True)
    view = core.view(100.1, 5.)
    assert not view['race']['recovery_ready'] and view['race']['resume_start_ready']
    assert view['controls']['resume_start']['allowed']
    assert core.submit('resume_start', 'ordinary', VIEW, 'c', None, 100.1)['stage'] == 'queued'
    denied = core_with(resume_start_ready=False)
    assert not denied.view(100.1, 5.)['controls']['resume_start']['allowed']


def test_events_note_decel_and_drain_rows_once():
    core = OpsCore('car1')
    core.update_status({**status(latch=None), 'speed_command_mps': 1.5, 'reason': 'a'}, 100.)
    core.update_status({**status(latch=None), 'speed_command_mps': .2, 'reason': 'follow_gap_hold'}, 100.2)
    assert any(e['kind'] == 'decel' for e in core.events)
    assert core.drain_rows() and not core.drain_rows()


def test_export_keeps_gaps_and_truth_separate(tmp_path):
    rows = [dict(row='sample', wall_s=1., sim_s=5., state='RUNNING', input_age_s=dict(scan=.05)),
            dict(row='event', wall_s=1.1, sim_s=5.1, kind='operator', text='<stop>', source='operator'),
            dict(row='sample', wall_s=3., sim_s=7., state='RACE_LATCHED', input_age_s=None)]
    table = race_ops_export.merge(rows, truth=[dict(t=5., progress_m=1.)])
    assert [row['sim_s'] for row in table['rows']] == [5., 5.1, 7.]
    assert table['gaps'] and table['gaps'][0]['gap_s'] == 2.
    csv = race_ops_export.render_csv(table)
    assert 'progress_m' not in csv and '0.05' in csv
    page = race_ops_export.render_html(table)
    assert '평가 정답' in page and '&lt;stop&gt;' in page and '<stop>' not in page
    assert len(race_ops_export.window(table['rows'], 5., 1., 1.)) == 2


def test_decision_rows_reuse_trace_and_mark_changes_only(tmp_path):
    path = tmp_path/'decisions.jsonl'
    calls = [dict(seq=i, kind='call', data=dict(input=dict(reference_s=float(i)), output=[1., 0., dict(reason=r, behavior='FOLLOW')]))
             for i, r in enumerate(('a', 'a', 'b'))]
    path.write_text('\n'.join(map(json.dumps, calls))+'\n{broken\n', encoding='utf-8')
    rows = race_ops_export.decision_rows(path, 0., 9.)
    assert [row['reason'] for row in rows] == ['a', 'b'] and rows[1]['sim_s'] == 2.
