"""평가 센서는 실행 사본의 관측만 추가하고 동역학은 변경하지 않는다."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('wheel_contact_launch', ROOT/'src/arena_bringup/launch/simulation.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)
sys.path.insert(0, str(ROOT/'scripts'))
from probe_braking_contact import flat_world, request_speed
from validate_g1_traffic import EvaluationCaptureGate


def model():
    return '<sdf><model name="arena_car">'+''.join(
        f'<link name="{axle}_{side}_wheel"><collision name="collision"><geometry><sphere><radius>0.0325</radius></sphere></geometry></collision><inertial><mass>.04</mass></inertial></link>'
        for axle in ('front', 'rear') for side in ('left', 'right'))+'</model></sdf>'


def test_contact_sensor_only_change():
    original = model()
    changed = ET.fromstring(launch._evaluation_wheel_contacts(original))
    links = changed.findall('model/link')
    assert len(links) == 4
    for link in links:
        sensor = link.find('sensor')
        assert sensor.findtext('contact/collision') == 'collision'
        assert sensor.findtext('contact/topic') == '/eval/wheels/contact'
        link.remove(sensor)
    assert ET.tostring(changed) == ET.tostring(ET.fromstring(original))


def test_missing_wheel_rejected():
    with pytest.raises(ValueError):
        launch._evaluation_wheel_contacts('<sdf><model name="arena_car"/></sdf>')


def test_duplicate_sensor_rejected():
    with pytest.raises(ValueError):
        launch._evaluation_wheel_contacts(launch._evaluation_wheel_contacts(model()))


def test_world_contact_plugin_only_once(tmp_path):
    source, first, second = [tmp_path/f'{name}.sdf' for name in ('source', 'first', 'second')]
    source.write_text('<sdf version="1.9"><world name="w"><model name="ground"/></world></sdf>')
    before = source.read_bytes()
    launch._runtime_world_overrides(source, first, 'configured', True, evaluation_wheel_contacts=True)
    launch._runtime_world_overrides(first, second, 'configured', True, evaluation_wheel_contacts=True)
    tree = ET.parse(second)
    assert len(tree.findall('world/plugin')) == 1
    assert tree.find('world/model').get('name') == 'ground'
    assert source.read_bytes() == before


def test_default_world_has_no_added_plugin(tmp_path):
    source, dest = tmp_path/'a.sdf', tmp_path/'b.sdf'
    source.write_text('<sdf><world name="w"/></sdf>')
    launch._runtime_world_overrides(source, dest, 'configured', True)
    assert not ET.parse(dest).findall('world/plugin')


@pytest.mark.parametrize('elapsed,speed', [(-1, 0), (0, 0), (.5, .5), (1, 1), (1.9, 1), (2, 1), (2.25, .5), (2.5, 0), (10, 0)])
def test_fixed_speed_profile(elapsed, speed):
    assert request_speed(elapsed) == pytest.approx(speed)


def test_flat_comparison_preserves_all_non_floor_collisions(tmp_path):
    source = ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf'
    before = source.read_bytes()
    output = tmp_path/'world.sdf'
    count = flat_world(source, output)
    old = ET.fromstring(before).find('world')
    new = ET.parse(output).getroot().find('world')
    original = {c.get('name'): c for c in old.findall('model/link/collision')}
    changed = {c.get('name'): c for c in new.findall('model/link/collision')}
    removed = set(original)-set(changed)
    assert len(removed) == count
    assert all(n == 'ground_plane_link__ground_collision' or n.startswith(('surface_', 'grass_')) for n in removed)
    for name in set(original)&set(changed):
        assert ET.tostring(original[name]) == ET.tostring(changed[name])
    floor = changed['collision']
    assert floor.findtext('pose') == '0 0 -0.047 0 0 0'
    assert floor.findtext('geometry/box/size') == '40 40 0.1'
    assert floor.find('surface') is None
    assert source.read_bytes() == before


def test_evaluation_capture_closes_before_serialization_tail():
    gate = EvaluationCaptureGate()
    captured = []
    accept = gate.wrap(captured.append)
    accept({'t': 1})
    gate.close()
    accept({'t': 2})
    assert captured == [{'t': 1}]


def test_evaluation_gate_waits_for_inflight_append():
    import threading
    gate, entered, release = EvaluationCaptureGate(), threading.Event(), threading.Event()
    captured = []
    def callback(value):
        entered.set()
        assert release.wait(2.)
        captured.append(value)
    receiver = threading.Thread(target=gate.wrap(callback), args=(1,))
    receiver.start()
    assert entered.wait(2.)
    closer = threading.Thread(target=gate.close)
    closer.start()
    release.set()
    receiver.join(2.)
    closer.join(2.)
    assert not receiver.is_alive() and not closer.is_alive()
    gate.wrap(captured.append)(2)
    assert captured == [1]
