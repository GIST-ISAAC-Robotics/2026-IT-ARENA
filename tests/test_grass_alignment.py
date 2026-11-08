"""단차 원인 대조는 새 실행 사본의 잔디 z만 바꾸며 기본 월드를 보존한다."""
import copy
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
from zipfile import ZipFile

import pytest

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf'
LAUNCH = 'src/arena_bringup/launch/simulation.launch.py'


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_only_grass_height_changes_and_source_is_unchanged(tmp_path):
    module = load(ROOT/LAUNCH, 'alignment_launch')
    before = WORLD.read_bytes()
    world = ET.parse(WORLD).find('world')
    original = copy.deepcopy(world)
    result = module._evaluation_grass_alignment(world, 'road_level')
    assert result['shifted_shapes'] == 952 and result['shift_z_m'] == .001
    assert result['after_top_m'] == .003
    old = original.find("model[@name='it_arena_track_static']/link[@name='grass']")
    new = world.find("model[@name='it_arena_track_static']/link[@name='grass']")
    for a, b in zip(old, new):
        assert float(b.find('pose').text.split()[2]) == .002
        b.find('pose').text = a.find('pose').text
    assert ET.tostring(world) == ET.tostring(original)
    assert WORLD.read_bytes() == before


def test_default_runtime_world_is_byte_identical_to_previous(tmp_path):
    module = load(ROOT/LAUNCH, 'alignment_launch_default')
    with ZipFile(ROOT/'artifacts/validation/2026-11-07/traffic_surface_alignment_v1/before.zip') as z:
        old_path = tmp_path/'previous.py'
        old_path.write_bytes(z.read(LAUNCH))
    old = load(old_path, 'alignment_launch_previous')
    a, b = tmp_path/'a.sdf', tmp_path/'b.sdf'
    old._runtime_world_overrides(WORLD, a, 'configured', True)
    module._runtime_world_overrides(WORLD, b, 'configured', True)
    assert a.read_bytes() == b.read_bytes()
    arguments = {v.name: v for v in module.generate_launch_description().get_launch_arguments()}
    assert arguments['evaluation_grass_alignment'].default_value[0].text == 'original'


@pytest.mark.parametrize('fault', ['height', 'missing', 'mode'])
def test_unregistered_geometry_or_mode_does_not_partially_mutate(fault):
    module = load(ROOT/LAUNCH, 'alignment_launch_invalid')
    world = ET.parse(WORLD).find('world')
    grass = world.find("model[@name='it_arena_track_static']/link[@name='grass']")
    if fault == 'height':
        grass[-1].find('geometry/box/size').text = '1 1 .004'
    if fault == 'missing':
        grass.set('name', 'different')
    before = ET.tostring(world)
    with pytest.raises(ValueError):
        module._evaluation_grass_alignment(world, 'arbitrary' if fault == 'mode' else 'road_level')
    assert ET.tostring(world) == before
