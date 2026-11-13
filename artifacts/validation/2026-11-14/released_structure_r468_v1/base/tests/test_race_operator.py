"""B3 운영 도구의 미전송 기본값·세대 고정·응답 판정. MCU/구동을 실행하지 않는다."""
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import race_operator as operator


def status(**kwargs):
    return dict(mode='b3_candidate', session='sessionA', race_epoch=2, intervention=5,
                latch='explicit_stop', started=True, rearm_required=True, **kwargs)


def command(op='acknowledge'):
    return operator.build_command(op, session='sessionA', race_epoch=2,
        **(dict(intervention=5, hazards_cleared=True, operator_reviewed=True) if op == 'acknowledge' else {}))


def test_dry_run_never_connects_and_has_no_mcu_commands(monkeypatch, capsys):
    monkeypatch.setattr(operator, 'exchange', lambda *a, **k: pytest.fail('must not connect'))
    assert operator.main(['stop']) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['outcome'] == 'dry_run_no_ROS_connection'
    assert set(report['command']) == {'v', 'request', 'op'}
    assert operator.main(['acknowledge', '--session', 'x', '--race-epoch', '0', '--intervention', '1']) == 2
    assert 'not_sent' in capsys.readouterr().out


@pytest.mark.parametrize('text', ['[]', '{"x":1,"x":2}', '{"x":NaN}', '{"x":Infinity}', 'x'*262145])
def test_invalid_json(text):
    with pytest.raises(ValueError):
        operator.decode(text)


@pytest.mark.parametrize('patch', [{'mode': 'legacy'}, {'session': ''}, {'race_epoch': True},
    {'intervention': -1}, {'started': 1}, {'rearm_required': None}])
def test_status_validation(patch):
    with pytest.raises(ValueError):
        operator.race_status({'race': {**status(), **patch}})


@pytest.mark.parametrize('kwargs', [dict(session='x', race_epoch=True), dict(session='x', race_epoch=-1),
    dict(session='bad id', race_epoch=0), dict(session='x'), dict()])
def test_new_race_requires_explicit_valid_tokens(kwargs):
    with pytest.raises(ValueError):
        operator.build_command('new_race', **kwargs)


def test_ack_flags_and_incompatible_tokens():
    for flag in ('hazards_cleared', 'operator_reviewed'):
        args = dict(session='x', race_epoch=0, intervention=1, hazards_cleared=True, operator_reviewed=True)
        args[flag] = False
        with pytest.raises(ValueError):
            operator.build_command('acknowledge', **args)
    with pytest.raises(ValueError):
        operator.build_command('stop', session='x')
    with pytest.raises(ValueError):
        operator.build_command('new_race', session='x', race_epoch=0, intervention=1)


@pytest.mark.parametrize('patch', [{'session':'old'}, {'race_epoch':3}, {'intervention':6}, {'latch':None}])
def test_stale_ack_never_retargets_current_state(patch):
    with pytest.raises(ValueError):
        operator.preflight(command(), {**status(), **patch})


def test_new_race_and_stop_preflight():
    operator.preflight(command('new_race'), status())
    operator.preflight(operator.build_command('stop'), None)


def test_matching_ack_requires_consistent_result():
    cmd = command()
    good = dict(v=1, request=cmd['request'], op='acknowledge', accepted=True, reason='ok',
                session='sessionA', race_epoch=2, intervention=5, latch=None)
    assert operator.matching_result(cmd, good)
    assert not operator.matching_result(cmd, {**good, 'request':'unrelated'})
    for patch in ({'accepted':1}, {'v':True}, {'session':'old'}, {'race_epoch':3}, {'intervention':6}, {'latch':'new_stop'}):
        with pytest.raises(ValueError):
            operator.matching_result(cmd, {**good, **patch})
    assert operator.matching_result(cmd, {**good, 'accepted':False, 'reason':'session_mismatch', 'session':'new'})


def test_new_race_result_must_increment_epoch_and_latch():
    cmd = command('new_race')
    good = dict(v=1, request=cmd['request'], op='new_race', accepted=True, reason='ok',
                session='sessionA', race_epoch=3, intervention=6, latch='new_race')
    assert operator.matching_result(cmd, good)
    with pytest.raises(ValueError):
        operator.matching_result(cmd, {**good, 'race_epoch':2})
    with pytest.raises(ValueError):
        operator.matching_result(cmd, {**good, 'latch':None})


def test_endpoint_identity_and_ambiguity():
    peer = SimpleNamespace(node_namespace='/', node_name='controller')
    mapping = {operator.COMMAND:[peer], operator.RESULT:[peer], operator.STATUS:[peer]}
    node = SimpleNamespace(get_subscriptions_info_by_topic=lambda t:mapping[t], get_publishers_info_by_topic=lambda t:mapping[t])
    assert operator.endpoints_ready(node)
    mapping[operator.STATUS] = []
    assert not operator.endpoints_ready(node)
    assert operator.endpoints_ready(node, stop_only=True)
    mapping[operator.RESULT] = [peer, peer]
    with pytest.raises(ValueError, match='ambiguous'):
        operator.endpoints_ready(node)
    mapping[operator.RESULT] = [SimpleNamespace(node_namespace='/', node_name='other')]
    mapping[operator.STATUS] = [peer]
    with pytest.raises(ValueError, match='different'):
        operator.endpoints_ready(node)


def test_cli_status_rejects_mutation_flags(monkeypatch, capsys):
    monkeypatch.setattr(operator, 'exchange', lambda *a, **k: pytest.fail('must not connect'))
    assert operator.main(['status', '--send']) == 2
    assert 'not_sent' in capsys.readouterr().out


def test_only_policy_publication_in_source():
    source = Path(operator.__file__).read_text()
    assert 'create_publisher(String, COMMAND' in source
    assert '/actuation/command' not in source and '/drive/safe' not in source


def test_real_supervisor_schema_accepts_generated_commands():
    from arena_autonomy.race_session import RaceSupervisor
    vision = SimpleNamespace(epoch=0, fault=None, signal=SimpleNamespace(started=False))
    def reset():
        vision.epoch += 1
        vision.signal = SimpleNamespace(started=False)
    vision.reset = reset
    supervisor = RaceSupervisor(vision, session='sessionA', now_s=1.)
    new = operator.build_command('new_race', session='sessionA', race_epoch=0)
    result = supervisor.handle_command(new, 2.)
    assert result['accepted'] and operator.matching_result(new, result)
    ack = operator.build_command('acknowledge', session='sessionA', race_epoch=1,
        intervention=result['intervention'], hazards_cleared=True, operator_reviewed=True)
    assert operator.matching_result(ack, supervisor.handle_command(ack, 2.1))
    stop = operator.build_command('stop')
    assert operator.matching_result(stop, supervisor.handle_command(stop, 2.2))
    assert not supervisor.decide(sensors_ready=True, now_s=2.3, wall_s=1.)['allowed']
