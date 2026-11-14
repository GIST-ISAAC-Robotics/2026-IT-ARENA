"""추가 actor 실행 경계의 ACK·자료 누락 회귀. Gazebo 실행은 별도다."""
import hashlib
from types import SimpleNamespace

import pytest

from test_traffic_actors import fixture, ta
from traffic_actor_runtime import ExtraActors


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def runtime(tmp_path):
    config, case = fixture()
    spec = ta.actor_specs(config, case)[1]
    extra = ExtraActors([spec], [0.], None, lambda *args: (0., 1., 0.), tmp_path, 0.)
    return config, spec, extra


def test_pending_rejection_cannot_complete_silently(tmp_path):
    _, spec, extra = runtime(tmp_path)
    name = spec['model_name']
    extra.records.commands[name].append({'requested_wall_absolute_s': 0., 'acknowledged': False})
    extra.pending[name] = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=False))
    with pytest.raises(RuntimeError, match=name):
        extra.complete()
    assert extra.pending[name] is None
    assert not extra.records.commands[name][0]['acknowledged']


def test_ready_requires_every_actor(tmp_path):
    _, spec, extra = runtime(tmp_path)
    assert not extra.ready
    extra.records.pose(spec['model_name'], 'it_arena_track', (0., 0., 1., 0.))
    assert extra.ready
    args, params = extra.bridge_arguments()
    assert len(args) == 1 and spec['model_name'] in args[0]
    assert any('reliability:=reliable' in p for p in params)


def test_actor_evaluation_requires_graph_and_dense_coverage(tmp_path):
    config, spec, extra = runtime(tmp_path)
    name = spec['model_name']
    poses = [(i*.01, 0., 1., 0.) for i in range(251)]
    extra.records.poses[name] = poses
    extra.reader_audit[name] = {'parse_errors': 0, 'handler_errors': 0}
    extra.subscribers[name] = ['g1b_traffic_validator']
    extra.connections[name] = {'returncode': 0, 'reader_alive_at_graph_audit': True,
        'output': 'Publishers\n tcp://127.0.0.1:1234 gz.msgs.Contacts\nSubscribers'}
    kwargs = dict(ego=[(t, x, 0., a) for t, x, y, a in poses], green=1., duration=1., config=config,
                  runtime_world=f'<sdf><model name="{name}"><static>true</static></model></sdf>',
                  known_contact_source=True, digest=digest)
    result = extra.save_and_evaluate(**kwargs)
    assert result['harness_valid'] and result['behavior_gates_passed']
    extra.subscribers[name].append('local_pursuit')
    assert not extra.save_and_evaluate(**kwargs)['harness_valid']
    extra.subscribers[name].pop()
    extra.records.poses[name] = poses[:110]+poses[140:]
    assert not extra.save_and_evaluate(**kwargs)['harness_valid']


def test_missing_green_or_actor_saves_failure_not_success(tmp_path):
    config, _, extra = runtime(tmp_path)
    result = extra.save_and_evaluate(ego=[], green=None, duration=1., config=config,
        runtime_world='', known_contact_source=True, digest=digest)
    assert not result['harness_valid'] and not result['behavior_gates_passed']
    assert (tmp_path/'additional_actors.json').exists()
