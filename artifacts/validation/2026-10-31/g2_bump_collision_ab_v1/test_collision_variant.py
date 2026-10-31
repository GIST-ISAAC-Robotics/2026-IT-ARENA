from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from collision_variant import remove_bump_collisions,collision_contract,shape

BASELINE=Path(__file__).resolve().parent.parent/'g2_motion_pause_level_v1/fixture/world.sdf'


def test_only_registered_collision_children_are_removed():
    old=ET.parse(BASELINE).getroot();new=deepcopy(old)
    assert remove_bump_collisions(new)==20
    oldlink=old.find(".//link[@name='safety_bump_0']");newlink=new.find(".//link[@name='safety_bump_0']")
    assert [shape(x) for x in oldlink if x.tag!='collision']==[shape(x) for x in newlink]
    for root in (old,new):root.find(".//model[@name='it_arena_track_static']").remove(root.find(".//link[@name='safety_bump_0']"))
    assert shape(old)==shape(new)


def test_runtime_refuses_collision_reintroduced_or_other_physics_changed(tmp_path):
    original=ET.parse(BASELINE).getroot();new=deepcopy(original);remove_bump_collisions(new)
    file=tmp_path/'world.sdf';ET.ElementTree(new).write(file)
    assert all(collision_contract(file,BASELINE).values())
    restored=deepcopy(new)
    restored.find(".//link[@name='safety_bump_0']").append(deepcopy(original.find(".//link[@name='safety_bump_0']/collision")))
    ET.ElementTree(restored).write(file)
    assert not all(collision_contract(file,BASELINE).values())
    changed=deepcopy(new);changed.find(".//link[@name='track_surface']/collision/geometry/box/size").text='1 1 1'
    ET.ElementTree(changed).write(file)
    assert not all(collision_contract(file,BASELINE).values())


def test_wrong_strip_count_and_absent_visual_fail(tmp_path):
    root=ET.parse(BASELINE).getroot();link=root.find(".//link[@name='safety_bump_0']")
    link.remove(link.find('collision'))
    with pytest.raises(ValueError,match='20-strip'):remove_bump_collisions(root)
    root=ET.parse(BASELINE).getroot();remove_bump_collisions(root)
    link=root.find(".//link[@name='safety_bump_0']")
    for visual in link.findall('visual'):link.remove(visual)
    file=tmp_path/'world.sdf';ET.ElementTree(root).write(file)
    assert not all(collision_contract(file,BASELINE).values())


def test_grass_height_and_track_pose_cannot_drift(tmp_path):
    root=ET.parse(BASELINE).getroot();remove_bump_collisions(root)
    file=tmp_path/'world.sdf'
    root.find(".//link[@name='grass']/pose").text='0 0 0 0 0 0'
    ET.ElementTree(root).write(file)
    assert not all(collision_contract(file,BASELINE).values())
    root.find(".//link[@name='grass']/pose").text='0 0 0.001 0 0 0'
    ET.SubElement(root.find(".//model[@name='it_arena_track_static']"),'pose').text='1 0 0 0 0 0'
    ET.ElementTree(root).write(file)
    assert not all(collision_contract(file,BASELINE).values())
