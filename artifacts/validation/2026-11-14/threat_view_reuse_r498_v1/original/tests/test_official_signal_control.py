"""공식 2×5 광원 제어의 이름·순서·경계. Gazebo 실제 전환은 render 산출물로 확인한다."""
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official'
CONTROLLER = ROOT/'src/arena_gazebo/scripts/traffic_light_controller.py'


@pytest.fixture(scope='module')
def controller():
    spec = importlib.util.spec_from_file_location('traffic_light_controller_test', CONTROLLER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_controller_lamps_are_the_official_links_and_channels(controller):
    model = ET.parse(WORLD/'world.sdf').find("./world/model[@name='it_arena_track_static']")
    for lamp in controller.LAMPS:
        assert model.find(f"link[@name='{lamp['name']}']/visual[@name='{lamp['name']}_vis']") is not None
    scene = json.loads((WORLD/'scene.json').read_text(encoding='utf-8'))
    assert sorted((v['name'], v['column']) for v in scene['traffic_light']['lamp_channels']) == \
        sorted((v['name'], v['column']) for v in controller.LAMPS)
    assert len(controller.LAMPS) == 10


@pytest.mark.parametrize('stage,reds,green', [('off', 0, False), ('green_probe', 0, True), ('red1', 1, False),
                                              ('red2', 2, False), ('red3', 3, False), ('red4', 4, False), ('green', 0, True)])
def test_each_stage_lights_expected_columns(controller, stage, reds, green):
    lit = {lamp['column'] for lamp in controller.LAMPS if controller.lamp_lit(stage, lamp['column'])}
    assert lit == set(range(reds)) | ({controller.GREEN_COLUMN} if green else set())


def test_green_never_shares_a_stage_with_red_and_start_is_red_to_green(controller):
    names = [name for name, _ in controller.sequence_stages(1.6)]
    assert names == ['green_probe', 'off', 'red1', 'red2', 'red3', 'red4', 'green']
    assert dict(controller.sequence_stages(1.6))['red4'] == 1.6
    assert controller.sequence_stages(1.6)[-1][1] is None
    assert controller.stage_label('red3') == 'red' and controller.stage_label('green_probe') == 'green_probe'


def test_lamp_request_targets_named_visual_and_turns_emission_off(controller):
    on = controller.lamp_request('lamp_green_top', 4, True)
    off = controller.lamp_request('lamp_red_2_bottom', 1, False)
    assert 'name:"lamp_green_top_vis" parent_name:"lamp_green_top"' in on and 'emissive {r:0 g:1 b:0 a:1}' in on
    assert 'name:"lamp_red_2_bottom_vis"' in off and 'emissive {r:0 g:0 b:0 a:1}' in off


def test_lit_lamps_are_applied_last_so_red_and_green_do_not_overlap(controller, monkeypatch):
    class Poll:
        __enter__ = lambda self: self
        __exit__ = lambda self, *_: False
        register = lambda self, *_: None
        select = lambda self, *_: True

    class Process:
        stdin = type('Stdin', (), dict(write=lambda self, text: None, flush=lambda self: None))()
        stdout = type('Stdout', (), dict(readline=lambda self: json.dumps(dict.fromkeys(
            ('parsed', 'executed', 'service_result', 'applied'), True))))()

    monkeypatch.setattr(controller.selectors, 'DefaultSelector', Poll)
    client = controller.LampClient.__new__(controller.LampClient)
    client.process = Process()
    assert [row['lit'] for row in client.apply('green')] == [False]*8+[True]*2


def test_vehicle_controller_never_reads_signal_or_simulator_state():
    for name in ('local_pursuit.py', 'vision_process.py', 'race_start.py', 'race_start_local.py'):
        text = (ROOT/'src/arena_autonomy/arena_autonomy'/name).read_text(encoding='utf-8')
        for forbidden in ('traffic_light/state', 'udp_port', '47810', 'dynamic_pose'):
            assert forbidden not in text


def test_official_launch_selects_rgb_2x5_start_only():
    text = (ROOT/'src/arena_bringup/launch/simulation.launch.py').read_text(encoding='utf-8')
    assert 'controller_parameters["start_signal_mode"] = "official_4red_local_candidate"' in text
    assert 'yellow_duration_s' not in text
    assert '신호 제어기는 공식 2×5 광원 링크가 있는 official 월드 전용입니다.' in text
