"""다중 상대와 접촉 플러그인을 유지하며 시설만 합치는지 확인한다."""
import xml.etree.ElementTree as ET
import pytest
from traffic_facility_fixture import merge

def test_actor_models_and_contact_plugin_are_preserved(tmp_path):
    raw=b'<sdf version="1.9"><plugin name="contact" filename="contact"/><model name="actor1"><static>true</static></model><model name="actor2"><static>true</static></model></sdf>'
    actor=tmp_path/'actors.sdf';actor.write_bytes(raw)
    output=tmp_path/'combined.sdf';evidence=merge(actor,output)
    root=ET.parse(output).getroot()
    assert actor.read_bytes()==raw
    assert root.find('plugin').get('name')=='contact'
    assert {m.get('name') for m in root.findall('model')}=={'actor1','actor2','eval_marker_20_supports','eval_marker_30_supports'}
    assert evidence['original_actors']==['actor1','actor2']
    assert not evidence['controller_truth_input']

def test_duplicate_support_models_and_existing_outputs_are_rejected(tmp_path):
    actor=tmp_path/'actor.sdf';actor.write_text('<sdf version="1.9"><model name="eval_marker_20_supports"/></sdf>')
    with pytest.raises(AssertionError):merge(actor,tmp_path/'new.sdf')
    assert not (tmp_path/'new.sdf').exists()
    actor.write_text('<sdf version="1.9"><model name="actor"/></sdf>')
    output=tmp_path/'old.sdf';output.write_text('preserved')
    with pytest.raises(AssertionError):merge(actor,output)
    assert output.read_text()=='preserved'

def test_launcher_has_one_combined_actor_input_and_evaluator_has_six_posts(tmp_path):
    from types import SimpleNamespace
    from pathlib import Path
    from traffic_posts_setup import configure
    from build_experimental_track import obstacle_polygons
    from traffic_facility_fixture import A
    root=A.parents[3]
    world=root/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf'
    original=world.read_bytes()
    calls=[]
    def start(*,extra_arguments=()):calls.append(extra_arguments);return 'started'
    target=SimpleNamespace(REPO=root,start_demo_process=start,obstacle_polygons=obstacle_polygons)
    original_count=len(obstacle_polygons(world))
    configure(target,tmp_path/'stage')
    assert len(target.obstacle_polygons(world))==original_count+6
    actor=tmp_path/'actors.sdf';actor.write_text('<sdf version="1.9"><plugin name="contact"/><model name="actor"/></sdf>')
    assert target.start_demo_process(extra_arguments=['evaluation_actor_sdf:='+str(actor)])=='started'
    assert len([a for a in calls[0] if a.startswith('evaluation_actor_sdf:=')])==1
    assert 'evaluation_course_markers:=blank_20_30' in calls[0]
    assert world.read_bytes()==original
    assert (tmp_path/'facility_merge.json').exists()
