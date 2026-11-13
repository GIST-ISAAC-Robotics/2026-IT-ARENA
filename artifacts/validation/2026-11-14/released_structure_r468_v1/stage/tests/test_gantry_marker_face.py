"""도형 비교본은 원본 마커와 충돌체를 변경하지 않습니다."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'scripts'))
from gantry_marker_face import add_sdf_marker_faces
from experimental_facilities import marker_cells


def source():
    return ET.parse(REPO / 'src/arena_gazebo/worlds/it_arena_official/world.sdf').find(
        "world/model[@name='it_arena_track_static']")


def test_faces_preserve_source_and_pattern():
    track = source()
    original = [ET.tostring(link) for link in track.findall('link')]
    collisions = len(track.findall('.//collision'))
    result = add_sdf_marker_faces(track)
    assert len(result) == 4
    assert len(track.findall('.//collision')) == collisions
    assert [ET.tostring(link) for link in track.findall('link')[:-4]] == original
    for item in result:
        link = track.find(f"link[@name='{item['link']}']")
        assert link.findtext('pose') == track.findtext(f"link[@name='aruco_{item['id']}']/pose")
        assert len(link.findall('visual')) == 1 + int((marker_cells(item['id']) == 0).sum())
        assert not link.findall('.//pbr')
        assert not link.findall('.//collision')
        for visual in link.findall('visual'):
            assert float(visual.findtext('pose').split()[0]) > .0025
            assert visual.findtext('cast_shadows') == 'false'


def test_duplicate_and_unexpected_geometry_do_not_partially_mutate():
    track = source()
    track.find("link[@name='aruco_45']/visual/geometry/box/size").text = '.005 .2 .2'
    before = ET.tostring(track)
    with pytest.raises(ValueError):
        add_sdf_marker_faces(track)
    assert ET.tostring(track) == before
    track = source()
    add_sdf_marker_faces(track)
    before = ET.tostring(track)
    with pytest.raises(ValueError):
        add_sdf_marker_faces(track)
    assert ET.tostring(track) == before
