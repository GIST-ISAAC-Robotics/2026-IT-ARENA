import json
import math
import xml.etree.ElementTree as ET

import pytest
import yaml
from shapely.geometry import box

from audit_grid_start import OFFICIAL, RUNTIME, VEHICLE, CAMERA, registry, measure, build_fixture


def inputs():
    return (json.loads((OFFICIAL/'scene.json').read_text()), (OFFICIAL/'world.sdf').read_bytes(),
            yaml.safe_load(VEHICLE.read_text())['vehicle'], yaml.safe_load(CAMERA.read_text())['camera'])


def test_registry_source_and_runtime_match_and_origin_is_not_rear_axle():
    scene, raw, vehicle, camera = inputs()
    rows = registry(scene, raw, vehicle)
    assert [r['index'] for r in rows] == [2, 5, 1, 4, 0, 3]
    assert rows == registry(json.loads((RUNTIME/'scene.json').read_text()), (RUNTIME/'world.sdf').read_bytes(), vehicle)
    assert rows[0]['spawn_model_origin'][:2] == [rows[0]['x'], rows[0]['y']]


@pytest.mark.parametrize('kind', ['duplicate', 'position', 'size', 'frame', 'nan'])
def test_bad_metadata_cannot_silently_make_a_valid_registry(kind):
    scene, raw, vehicle, camera = inputs()
    if kind == 'duplicate':
        scene['starting_grid']['slots'][1]['index'] = 0
    elif kind == 'position':
        scene['starting_grid']['slots'][0]['x'] += .01
    elif kind == 'size':
        scene['starting_grid']['slot_width_m'] = .25
    elif kind == 'frame':
        vehicle['reference_frame'] = 'rear_axle'
    else:
        scene['starting_grid']['slots'][0]['yaw_rad'] = math.nan
    with pytest.raises(ValueError):
        registry(scene, raw, vehicle)


def test_merged_visual_pose_mismatch_and_duplicate_are_rejected():
    scene, raw, vehicle, camera = inputs()
    tree = ET.fromstring((RUNTIME/'world.sdf').read_bytes())
    visual = tree.find(".//visual[@name='grid_slot_0__grid_slot_0_vis']")
    values = list(map(float, visual.findtext('pose').split()))
    values[0] += .02
    visual.find('pose').text = ' '.join(map(str, values))
    with pytest.raises(ValueError, match='mismatch'):
        registry(scene, ET.tostring(tree), vehicle)
    tree = ET.fromstring(raw)
    tree.find('world/model').append(ET.fromstring(ET.tostring(tree.find(".//link[@name='grid_slot_0']"))))
    with pytest.raises(ValueError, match='ambiguous'):
        registry(scene, ET.tostring(tree), vehicle)


def test_touch_or_overlap_and_road_exit_are_distinct_from_clearance():
    rows = [dict(index=i, x=float(i)*.20, y=0., yaw_rad=0.) for i in range(6)]
    result = measure(rows, box(-10, -10, 10, 10), [], .2, .15)
    assert result['subsets']['2']['no_overlap']
    assert result['subsets']['2']['minimum_gap_m'] == pytest.approx(0.)
    assert not result['subsets']['2']['meets_existing_traffic_gap']
    rows[1]['x'] = .1
    result = measure(rows, box(-.1, -.075, .1, .075), [box(0., -.1, .3, .1)], .2, .15)
    assert not result['subsets']['2']['no_overlap']
    assert not result['subsets']['2']['contained_on_road']
    assert not result['subsets']['2']['no_low_wall_overlap']


def test_fixture_preserves_official_lights_and_geometry_and_has_no_drive_plugins():
    scene, raw, vehicle, camera = inputs()
    root, views = build_fixture(raw, registry(scene, raw, vehicle), vehicle, camera,
                                dict(bottom_m=0., height_m=.12))
    world = root.find('world')
    assert len(world.findall('model')) == 1+5+3
    assert set(views) == {'slot2', 'slot0', 'overview'}
    original = ET.fromstring(raw).find('world/model')
    track = world.find('model')
    for name in ('lamp_red_1', 'lamp_red_4', 'lamp_green', 'track_surface', 'grid_slot_0'):
        path = f"link[@name='{name}']"
        assert ET.tostring(original.find(path)) == ET.tostring(track.find(path))
    assert not any('Drive' in p.get('name', '') or 'actuation' in p.get('name', '') for p in world.iter('plugin'))
    assert all(m.findtext('static') == 'true' for m in world.findall('model'))
