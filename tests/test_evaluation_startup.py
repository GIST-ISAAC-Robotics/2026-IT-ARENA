"""초기 차량/paused 시작의 평가 전용 범위·준비 거부·기본 보존 회귀."""
import copy
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import xml.etree.ElementTree as ET
import zipfile

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import evaluation_startup as gate
spec = importlib.util.spec_from_file_location('startup_launch', ROOT/'src/arena_bringup/launch/simulation.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)
WORLD = '<sdf version="1.10"><world name="it_arena_track"><physics name="p"><max_step_size>0.001</max_step_size></physics><model name="wall"><static>true</static></model></world></sdf>'
MODEL = '<sdf version="1.10"><model name="arena_car"><link name="body"><inertial><mass>2</mass></inertial></link></model></sdf>'
INFO = 'Publishers [Address, Message Type]:\n  tcp://127.0.0.1:123, gz.msgs.StringMsg\nSubscribers [Address, Message Type]:\n  tcp://127.0.0.1:456, gz.msgs.StringMsg\n'
PAUSED = {'paused': True, 'iterations': 0, 'sim_time': {'sec': 0, 'nsec': 0}}
CONTACT_INFO = INFO.replace('  tcp://127.0.0.1:123, gz.msgs.StringMsg\n', '  tcp://127.0.0.1:123, gz.msgs.Contacts\n'*4)


def info_for(args):
    return CONTACT_INFO if '/eval/wheels/contact' in args else INFO


def test_embedded_only_adds_model_and_pose(tmp_path):
    world = tmp_path/'world.sdf'
    world.write_text(WORLD)
    launch._embed_evaluation_vehicle(world, MODEL, 2., 3., .5)
    tree = ET.parse(world).getroot()
    model = tree.find('world/model[@name="arena_car"]')
    assert model.findtext('pose') == '2.0 3.0 0.01 0 0 0.5'
    model.remove(model.find('pose'))
    assert ET.tostring(model) == ET.tostring(ET.fromstring(MODEL).find('model'))
    tree.find('world').remove(model)
    assert ET.tostring(tree) == ET.tostring(ET.fromstring(WORLD))


@pytest.mark.parametrize('model', [MODEL.replace('arena_car', 'other'), MODEL.replace('</model>', '<pose>0 0 0 0 0 0</pose></model>'), '<sdf/>'])
def test_bad_model_rejected_without_write(tmp_path, model):
    world = tmp_path/'world.sdf'; world.write_text(WORLD)
    with pytest.raises(ValueError): launch._embed_evaluation_vehicle(world, model, 0., 0., 0.)
    assert world.read_text() == WORLD


@pytest.mark.parametrize('world_text', [WORLD.replace('wall', 'arena_car'), WORLD.replace('</world>', '<include><uri>other</uri></include></world>')])
def test_duplicate_or_include_rejected(tmp_path, world_text):
    world = tmp_path/'world.sdf'; world.write_text(world_text)
    with pytest.raises(ValueError): launch._embed_evaluation_vehicle(world, MODEL, 0., 0., 0.)
    assert world.read_text() == world_text


@pytest.mark.parametrize('index', range(3))
def test_nonfinite_pose_rejected(tmp_path, index):
    p = tmp_path/'w.sdf'; p.write_text(WORLD)
    xyz = [0., 0., 0.]; xyz[index] = float('nan')
    with pytest.raises(ValueError): launch._embed_evaluation_vehicle(p, MODEL, *xyz)
    assert p.read_text() == WORLD


def test_default_flags_and_arguments_unchanged():
    assert launch._evaluation_server_flags('service') == ['-r']
    assert launch._evaluation_server_flags('embedded_paused') == []
    assert gate.startup_arguments('service', False, False) == []
    launch._check_evaluation_startup('service', '', False, False, False, True)
    assert gate.startup_arguments('embedded_paused', True, True) == ['evaluation_startup:=embedded_paused']


@pytest.mark.parametrize('index,value', [(0, ''), (1, False), (2, False), (3, False), (4, True)])
def test_restricted_launch_options(index, value):
    options = ['evidence', True, True, True, False]; options[index] = value
    with pytest.raises(ValueError): launch._check_evaluation_startup('embedded_paused', *options)


@pytest.mark.parametrize('grid,trace', [(False, False), (False, True), (True, False)])
def test_gate_requires_grid_trace(grid, trace):
    with pytest.raises(ValueError): gate.startup_arguments('embedded_paused', grid, trace)


def test_default_world_matches_previous(tmp_path):
    before = ROOT/'artifacts/validation/2026-10-11/evaluator_gc_v1/b2_defer/source_snapshot.zip'
    old_path = tmp_path/'old.py'
    with zipfile.ZipFile(before) as z: old_path.write_bytes(z.read('src/arena_bringup/launch/simulation.launch.py'))
    spec = importlib.util.spec_from_file_location('old_startup_launch', old_path)
    old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)
    source = ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf'
    old._runtime_world_overrides(source, tmp_path/'a.sdf', 'configured', True, False, False, None, True)
    launch._runtime_world_overrides(source, tmp_path/'b.sdf', 'configured', True, False, False, None, True)
    assert (tmp_path/'a.sdf').read_bytes() == (tmp_path/'b.sdf').read_bytes()


def test_info_parser_fail_closed():
    assert gate.topic_connections(INFO) == {'publishers': 1, 'subscribers': 1}
    assert gate.topic_connections('No publishers on topic\nNo subscribers on topic') == {'publishers': 0, 'subscribers': 0}
    assert gate.topic_connections('tcp://without-section') == {'publishers': 0, 'subscribers': 0}


class FakeNode:
    count = 1
    def count_publishers(self, topic): return self.count
    def get_subscriptions_info_by_topic(self, topic): return [SimpleNamespace(node_name='lidar_safety')]
    def get_publishers_info_by_topic(self, topic): return [SimpleNamespace(node_name='lidar_safety')]
    def service_is_ready(self): return True


@pytest.fixture
def ready(monkeypatch):
    partition = f'arena_autonomy_test_{os.getpid()}'
    monkeypatch.setenv('GZ_PARTITION', partition)
    calls = []
    def run(args, **kwargs):
        calls.append(args)
        return SimpleNamespace(returncode=0, stdout=info_for(args) if args[1] == 'topic' else 'data: true\n', stderr='')
    clock = [0.]
    startup = gate.PausedStartup(partition, runner=run, clock=lambda: clock[0])
    startup.observe(copy.deepcopy(PAUSED)); startup.observe(copy.deepcopy(PAUSED))
    return startup, FakeNode(), calls, clock


def test_release_only_after_readiness_and_exact_service(ready):
    s, n, calls, _ = ready
    assert s.poll(n, n, n, 0, [])
    assert s.record['released'] and calls[-1][-1] == 'pause: false'
    assert len(calls) == 3 and s.record['pre_release_sim_ns'] == 0
    assert s.poll(n, n, n, 1_000_000, [{'x': 1}])
    assert len(calls) == 3


@pytest.mark.parametrize('count', [0, 2])
def test_graph_not_ready_no_unpause(ready, count):
    s, n, calls, _ = ready; n.count = count
    assert not s.poll(n, n, n, 0, []) and not calls


def test_stats_missing_no_unpause(ready):
    s, n, calls, _ = ready; s.stats.clear()
    assert not s.poll(n, n, n, 0, []) and not calls


@pytest.mark.parametrize('what', ['clock', 'trace', 'stats', 'timeout'])
def test_early_start_and_timeout_fail_closed(ready, what):
    s, n, calls, clock = ready
    if what == 'stats': s.stats[0]['paused'] = False
    if what == 'timeout': clock[0] = 91
    with pytest.raises((RuntimeError, TimeoutError)):
        s.poll(n, n, n, 1 if what == 'clock' else 0, [{}] if what == 'trace' else [])
    assert not calls


def test_transport_absent_no_unpause(ready):
    s, n, calls, _ = ready
    s.runner = lambda *a, **kw: SimpleNamespace(returncode=0, stdout='', stderr='')
    assert not s.poll(n, n, n, 0, []) and not s.record['released']


def test_service_rejection_not_success(ready):
    s, n, _, _ = ready
    s.runner = lambda args, **kw: SimpleNamespace(returncode=0, stdout=info_for(args) if args[1] == 'topic' else 'data: false', stderr='')
    with pytest.raises(RuntimeError): s.poll(n, n, n, 0, [])
    assert not s.record['released']


def test_first_update_audit_rejects_late_spawn(ready):
    s, n, _, _ = ready; s.poll(n, n, n, 0, [])
    event = {'type': 'first_update', 'event_seq': 1, 'iteration': 1, 'sim_ns': 1_000_000, 'dt_ns': 1_000_000}
    assert gate.audit_startup(s.record, [event])['passed']
    event['iteration'] = 2
    assert not gate.audit_startup(s.record, [event])['passed']
    assert not gate.audit_startup(s.record, [])['passed']


def test_pre_release_evidence_frozen_during_service(ready):
    s, n, _, _ = ready
    def run(args, **kw):
        if args[1] == 'service': s.observe({'paused': False, 'iterations': 1})
        return SimpleNamespace(returncode=0, stdout=info_for(args) if args[1] == 'topic' else 'data: true', stderr='')
    s.runner = run
    assert s.poll(n, n, n, 0, [])
    assert len(s.record['stats_before_release']) == 2


def test_contact_publisher_count_matches_actual_generated_model():
    model = ET.parse(ROOT/'artifacts/validation/2026-10-11/evaluator_gc_v1/b2_defer/runtime_vehicle.sdf')
    sensors = model.findall('.//sensor[@type="contact"]')
    assert len([s for s in sensors if s.findtext('contact/topic') == '/eval/wheels/contact']) == 4
    assert gate.EXPECTED_GZ_PUBLISHERS['/eval/wheels/contact'] == 4


@pytest.mark.parametrize('count', [1, 3, 5])
def test_wrong_contact_count_rejected(ready, count):
    s, n, _, _ = ready
    info = INFO.replace('  tcp://127.0.0.1:123, gz.msgs.StringMsg\n', '  tcp://127.0.0.1:123, gz.msgs.Contacts\n'*count)
    s.runner = lambda args, **kw: SimpleNamespace(returncode=0,
        stdout=info if '/eval/wheels/contact' in args else INFO, stderr='')
    assert not s.poll(n, n, n, 0, [])
    assert not s.record.get('release_requested')
