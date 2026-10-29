import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import run_onroad as run

ROOT=run.ROOT
PARENT=ROOT/'artifacts/validation/2026-10-29/g2_openframe_v1'
spec=importlib.util.spec_from_file_location('known_shape_fixture',PARENT/'test_shape.py')
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)

def test_shape_copy_matches_tested_six_part_collision():
    assert (run.BASE/'shape.py').read_bytes()==(PARENT/'shape.py').read_bytes()
    assert json.loads((PARENT/'contact_probe1/report.json').read_text())['passed'] is True

def test_side_only_replaces_all_six_contact_topics_preserving_pose_and_publisher():
    actor=dict(fixture.ACTOR,role='side',contact_topic='/eval/traffic/eval_frame/contact')
    original=fixture.sample()
    transformed=run.transform_actor(original,actor)
    before=ET.fromstring(original);after=ET.fromstring(transformed)
    assert before.findtext('model/pose')==after.findtext('model/pose')
    assert [ET.tostring(x) for x in before.findall('model/plugin')]==[ET.tostring(x) for x in after.findall('model/plugin')]
    sensors=after.findall('model/link/sensor')
    assert len(sensors)==6 and {x.findtext('contact/topic') for x in sensors}=={actor['contact_topic']}
    assert run.shape_audit(transformed,actor)['passed'] is True
    assert run.transform_actor(original,dict(actor,role='lead'))==original

def test_original_schedules_and_projected_dimensions_unchanged():
    for side,name in [('left','e3_onroad_encounter_v1.json'),('right','e3_onroad_right_encounter_v1.json')]:
        old=json.loads((ROOT/'config/tests'/name).read_text())['additional_actors'][0]
        new=json.loads((run.BASE/(side+'_overlay.json')).read_text())['additional_actors'][0]
        assert all(new[k]==v for k,v in old.items())
        assert (new['length_m'],new['width_m'],new['height_m'],new['bottom_m'])==(.2,.15,.14,0.)
