"""코스 마커 무늬 누락 평가: 제어기에 누락/정답을 주지 않고 표시 재질만 바꾼다."""
import copy
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import pytest
from test_g1_solo_candidate_validation import helpers

ROOT=Path(__file__).resolve().parents[1]
PATH=ROOT/'src/arena_bringup/launch/simulation.launch.py'
OFFICIAL=ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf'
spec=importlib.util.spec_from_file_location('marker_visibility_launch',PATH)
launch=importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


@pytest.mark.parametrize('mode,ids',[('blank_30',[30]),('blank_all',[0,20,30,45])])
def test_only_selected_visual_materials_change(mode,ids):
    before=OFFICIAL.read_bytes()
    root=ET.fromstring(before);original=copy.deepcopy(root)
    result=launch._evaluation_course_markers(root.find('world'),mode)
    assert result['hidden_ids']==ids
    for marker in ids:
        selector=f".//link[@name='aruco_{marker}']/visual"
        visual=root.find(selector);material=visual.find('material')
        assert material.find('.//albedo_map') is None
        assert material.findtext('diffuse')=='.4 .4 .4 1'
        position=list(visual).index(material)
        visual.remove(material)
        visual.insert(position,copy.deepcopy(original.find(selector+'/material')))
    assert ET.tostring(root)==ET.tostring(original)
    assert OFFICIAL.read_bytes()==before


def test_visible_is_noop_and_unknown_or_missing_marker_is_rejected():
    world=ET.fromstring('<world name="w"/>')
    original=ET.tostring(world)
    assert launch._evaluation_course_markers(world,'visible')['hidden_ids']==[]
    assert ET.tostring(world)==original
    for bad in ('all','blank_49','blank_all'):
        with pytest.raises(ValueError):launch._evaluation_course_markers(world,bad)


def test_default_runtime_world_matches_successful_lap_bytes(tmp_path):
    archive=ROOT/'artifacts/validation/2026-10-27/partial_observation_v1/run10_source_snapshot.zip'
    path=tmp_path/'before.py'
    with zipfile.ZipFile(archive) as saved:path.write_bytes(saved.read(PATH.relative_to(ROOT).as_posix()))
    spec=importlib.util.spec_from_file_location('marker_before_launch',path)
    previous=importlib.util.module_from_spec(spec);spec.loader.exec_module(previous)
    previous._runtime_world_overrides(OFFICIAL,tmp_path/'old.sdf','configured',True,False)
    launch._runtime_world_overrides(OFFICIAL,tmp_path/'new.sdf','configured',True,False)
    assert (tmp_path/'old.sdf').read_bytes()==(tmp_path/'new.sdf').read_bytes()


def test_route_still_requires_progress_and_other_marker_evidence():
    route=helpers()['solo_route_complete']
    assert not route(40.,48.,True,set(),'blank_all')
    assert route(49.,48.,True,set(),'blank_all')
    assert not route(49.,48.,True,set(),'visible')
    assert not route(49.,48.,True,set(),'blank_30')
    assert route(49.,48.,True,{20},'blank_30')


def test_nontextured_or_duplicate_marker_cannot_be_silently_claimed_hidden():
    world=ET.fromstring('<world><model><link name="aruco_30"><visual><material/></visual></link></model></world>')
    with pytest.raises(ValueError):launch._evaluation_course_markers(world,'blank_30')
    world.find('model').append(copy.deepcopy(world.find('model/link')))
    with pytest.raises(ValueError):launch._evaluation_course_markers(world,'blank_30')
