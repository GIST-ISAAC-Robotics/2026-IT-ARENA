"""성능 옵션이 운영 원본·충돌 형상·물성을 보존하는지 확인합니다."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("runtime_world_launch", ROOT / "src/arena_bringup/launch/simulation.launch.py")
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


def test_runtime_detector_preserves_original_and_collision_properties(tmp_path):
    source = ROOT / "src/arena_gazebo/worlds/it_arena_official/world.sdf"
    before = source.read_bytes()
    output = tmp_path / "world.sdf"
    launch._runtime_world_overrides(source, output, "bullet", False)
    original = ET.fromstring(before).find("world")
    changed = ET.parse(output).getroot().find("world")
    assert source.read_bytes() == before
    assert changed.findtext("physics/max_step_size") == original.findtext("physics/max_step_size")
    assert changed.findtext("physics/real_time_factor") == original.findtext("physics/real_time_factor")
    assert changed.findtext("physics/dart/collision_detector") == "bullet"
    assert not changed.findall("plugin[@name='gz::sim::systems::SceneBroadcaster']")
    for old, new in zip(original.findall(".//link"), changed.findall(".//link")):
        assert old.attrib == new.attrib
        assert old.findtext("pose") == new.findtext("pose")
    old_collisions = original.findall(".//collision")
    new_collisions = changed.findall(".//collision")
    assert len(old_collisions) == len(new_collisions) == 3740
    for old, new in zip(old_collisions, new_collisions):
        for uri in old.findall(".//uri"):
            if uri.text and "://" not in uri.text and not Path(uri.text).is_absolute():
                uri.text = str((source.parent / uri.text).resolve())
        assert ET.tostring(old) == ET.tostring(new)


def test_runtime_configured_keeps_detector_and_scene(tmp_path):
    source = tmp_path / "input.sdf"
    source.write_text('<sdf version="1.8"><world name="test"><physics name="p" type="ode"><dart><collision_detector>fcl</collision_detector></dart></physics><plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/></world></sdf>')
    output = tmp_path / "output.sdf"
    launch._runtime_world_overrides(source, output, "configured", True)
    tree = ET.parse(output)
    assert tree.findtext("world/physics/dart/collision_detector") == "fcl"
    assert tree.find("world/plugin") is not None
