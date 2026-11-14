"""B3 실제 ROS 시험기의 픽셀/사례 구성 회귀. ROS 실행 결과와는 별개다."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import validate_pipeline_faults_ros as validation
from arena_autonomy.race_start import OfficialStartSignal


def test_runtime_manifest_covers_static_local_imports_transitively():
    # 모듈을 추가할 때 실행 경로에 import되지만 보존 목록에는 없는 파일을 방지한다.
    # 동적 import 문자열/외부 라이브러리 전체의 재현성 검사는 아니다.
    import ast
    manifest = {f'{package}.{name}' for package, names in validation.MODULES.items() for name in names}
    for module in manifest:
        package, name = module.split('.')
        tree = ast.parse((ROOT/'src'/package/package/f'{name}.py').read_text(encoding='utf-8'))
        imports = []
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module:
                imports.append(node.module)
            elif isinstance(node, ast.Import):
                imports.extend(alias.name for alias in node.names)
        required = {name for name in imports if name.startswith(('arena_autonomy.', 'arena_vehicle_interface.'))}
        assert required <= manifest, (module, sorted(required-manifest))


def test_official_fixture_uses_pixel_sequence_not_side_channel():
    detector = OfficialStartSignal()
    stamp = 0.
    for color in ('green', 'off', 'red1', 'red2', 'red3', 'red', 'green'):
        rgb = np.frombuffer(validation.signal_rgb(color, 'official_4red_candidate'), np.uint8).reshape(480, 848, 3)
        for _ in range(3):
            stamp += .1
            detector.update(rgb, stamp)
        if stamp < 2.:
            assert not detector.started
    assert detector.started


def test_fixture_unknown_mode_fails():
    with pytest.raises(ValueError, match='mode'):
        validation.signal_rgb('red', 'unsupported')


def test_official_ros_sequence_retains_independent_stop_and_rearm():
    validator = validation.PipelineValidator.__new__(validation.PipelineValidator)
    validator.signal_mode = 'official_4red_candidate'
    events = []
    validator.call = lambda operation: events.append(('service', operation))
    validator.children = SimpleNamespace(send=lambda *args, **kw: events.append(('exit', 'controller')))
    validator.stop_case = lambda name, action, expected, **kw: (events.append(('stop', name, kw)), action())
    validator.moving = lambda name: events.append(('move', name))
    validator.red_green = lambda: events.append(('pixels', 'red_green'))
    validator.restart_controller = lambda: events.append(('restart', 'no_preserved_start'))
    validator.final_stop = lambda: events.append(('final', 'stop_graph_cleanup'))
    validator.run_start_contract()
    names = [e[1] for e in events]
    assert names.index('stop') < names.index('clear_stop') < names.index('arm')
    assert names[-1] == 'stop_graph_cleanup'
    assert names.index('official_controller_exit_stops_before_restart') < names.index('no_preserved_start')
    assert 'official_green_extinguished_keeps_race_start' in names
    assert 'official_fresh_off_image_restores_existing_permit' in names
    assert all(e[2]['no_resume'] for e in events if e[0] == 'stop' and
               (e[1].endswith('_no_start') or e[1] == 'official_clear_alone_is_disarmed'))


def test_official_scenario_cannot_silently_use_old_detector():
    validator = validation.PipelineValidator.__new__(validation.PipelineValidator)
    validator.signal_mode = 'temporary_3light'
    with pytest.raises(ValueError, match='official candidate'):
        validator.run_start_contract()


@pytest.mark.parametrize('failed', [False, True])
def test_failure_cleanup_stop_does_not_relabel_the_failed_run(failed):
    validator = validation.PipelineValidator.__new__(validation.PipelineValidator)
    validator.report = {'passed': False, 'error': 'original failure'}
    validator.monitor = SimpleNamespace(zero_watch=object(), sequence=7)
    def call(name):
        assert name == 'stop'
        if failed:
            raise RuntimeError('stop unreachable')
    validator.call = call
    validator.stopped = lambda seq: seq == 7
    validator.wait = lambda predicate, *a, **kw: predicate()
    validator.stop_after_failure()
    assert validator.report['passed'] is False
    assert validator.report['error'] == 'original failure'
    assert validator.report['failure_stop']['acknowledged'] is not failed
    assert validator.report['failure_stop']['fresh_zero_and_model_stopped'] is not failed


def test_race_session_v2_sequence_handshakes_stop_ids_and_delayed_states():
    validator = validation.PipelineValidator.__new__(validation.PipelineValidator)
    validator.signal_mode = 'official_4red_candidate'
    events = []
    state = dict(session='s1', race_epoch=0, intervention=1, latch='process_start', rearm_fence_s=1.,
                 mcu_counts={}, counts=dict(rearm_ignored_before_fence=0), rearm_baseline={'b': 1},
                 mcu=dict(generation=3))
    requests = iter(range(1, 1000))

    def race_state():
        return dict(state)

    def command(op, **fields):
        request = 'validator-%d' % next(requests)
        events.append(('race', op, request, fields))
        if op == 'stop':
            state['intervention'] += 1
            return dict(accepted=True, reason='stop_latched', request=request)
        if op == 'new_race':
            ok = fields.get('session') == state['session'] and fields.get('race_epoch') == state['race_epoch']
            if ok:
                state['race_epoch'] += 1
                state['intervention'] += 1
            return dict(accepted=ok, reason='new_race' if ok else 'race_epoch_mismatch', request=request)
        ok = fields == dict(session=state['session'], race_epoch=state['race_epoch'],
                            intervention=state['intervention'], hazards_cleared=True, operator_reviewed=True)
        return dict(accepted=ok, request=request, reason='policy_latch_cleared' if ok else
                    ('session_mismatch' if fields.get('session') != state['session'] else 'race_epoch_mismatch'))

    def send(payload, request=None):
        events.append(('send', payload if isinstance(payload, str) else payload['request']))
        if isinstance(payload, str):
            return dict(accepted=False, reason='invalid_payload')
        return dict(accepted=True, reason='stop_latched', request=request)

    def call(name):
        events.append(('service', name))
        if validator.relay_hold is not None and name in ('disarm', 'arm'):
            validator.relay_hold.append(SimpleNamespace(data=json.dumps(dict(
                state='DISARMED' if name == 'disarm' else 'ARMED', generation=4 if name == 'disarm' else 5))))
        if name in ('stop', 'reset'):
            state['intervention'] += 1
        if name == 'reset':
            state['race_epoch'] += 1

    def children(kind):
        events.append((kind, 'controller'))
        if kind == 'start':
            state.update(session='s2', intervention=1)

    validator.race_state = race_state
    validator.race_command = command
    validator.race_send = send
    validator.enable = lambda value: SimpleNamespace(success=not value, message='B3: enable cannot release')
    validator.call = call
    validator.call_result = lambda name: SimpleNamespace(success=False, message='not_disarmed')
    validator.expect = lambda result, accepted, reason=None: events.append(('expect', accepted, reason)) or (
        result if isinstance(result, dict) else dict(request=None))
    validator.children = SimpleNamespace(register_vision=lambda pid: 'owned',
        send=lambda *args, **kw: events.append(('kill', 'vision')),
        stop=lambda role: children('stop'), start=lambda role: children('start'))
    validator.data = lambda key: {'vision': {'worker_pid': 1}}
    validator.stop_case = lambda name, action, expected, **kw: (events.append(('stop', name, kw)), action())
    validator.moving = lambda name: events.append(('move', name))
    validator.red_green = lambda name='x', expect_motion=True: events.append(('pixels', name))
    validator.pump = validator.wait = lambda *a, **kw: None
    validator.monitor = SimpleNamespace(sequence=0, events=[])
    validator.report = {}
    validator.relay_hold = None
    validator.release_relay = lambda: (events.append(('relay', 'release')), setattr(validator, 'relay_hold', None))
    validator.last = {'image': 1.}
    validator.final_stop = lambda: events.append(('final', 'stop_graph_cleanup'))
    validator.run_race_session()
    names = [e[1] for e in events]
    assert (names.index('b3_startup_lights_without_ack_no_start') <
            names.index('b3_startup_ack_with_post_start_arm_and_lights'))
    upper = names.index('b3_upper_stop_policy_latch_mcu_still_armed')
    first_stop = next(e[2] for e in events if e[0] == 'race' and e[1] == 'stop')
    ack_ids = {e[2] for e in events[upper:] if e[0] == 'race' and e[1] == 'acknowledge'}
    sends = [e[1] for e in events if e[0] == 'send']
    # 활성 STOP 재전송, ack 번호와 겹친 STOP, 해제된 STOP 번호 재전송
    assert sends.count(first_stop) == 2 and any(request in ack_ids for request in sends)
    delayed = names.index('b3_delayed_pre_latch_disarm_arm_not_rearm')
    order = [e[1] for e in events[delayed:] if e[0] in ('service', 'race', 'relay')]
    assert order[:4] == ['disarm', 'arm', 'stop', 'release']
    assert validator.report['delayed_status_injection']['held_states'] == ['ARMED', 'DISARMED']
    assert (names.index('b3_new_race_latches_and_requires_new_sequence') <
            names.index('b3_new_race_ack_and_lights_still_need_rearm') <
            names.index('b3_new_race_start_after_ack_rearm_and_lights'))
    restart = names.index('b3_restart_requires_ack_and_new_arm_not_previous_armed')
    assert (names.index('b3_vision_fault_ack_rejected') < restart <
            names.index('b3_restart_resume_after_ack_rearm_and_lights'))
    assert names[-1] == 'stop_graph_cleanup'
    no_resume = {e[1] for e in events if e[0] == 'stop' and len(e) == 3 and e[2].get('no_resume')}
    assert {'b3_startup_lights_without_ack_no_start', 'b3_ack_without_mcu_rearm_no_resume',
            'b3_delayed_pre_latch_disarm_arm_not_rearm', 'b3_mcu_clear_arm_without_ack_no_resume',
            'b3_new_race_green_only_no_start', 'b3_new_race_ack_and_lights_still_need_rearm',
            'b3_legacy_reset_ack_and_lights_still_need_rearm',
            'b3_restart_requires_ack_and_new_arm_not_previous_armed'} <= no_resume


def test_race_session_scenario_cannot_use_old_detector():
    validator = validation.PipelineValidator.__new__(validation.PipelineValidator)
    validator.signal_mode = 'temporary_3light'
    with pytest.raises(ValueError, match='official candidate'):
        validator.run_race_session()
