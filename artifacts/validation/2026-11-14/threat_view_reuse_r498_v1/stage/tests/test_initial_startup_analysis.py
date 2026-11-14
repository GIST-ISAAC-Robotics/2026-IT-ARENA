import copy
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from analyze_initial_startup import normalized_xml, world_and_model, readiness_evidence, compare, bounded_confirmation

MODEL = '<model name="arena_car"><link name="b"><inertial><mass>2</mass></inertial></link></model>'
WORLD = '<sdf><world name="w"><model name="wall"><link name="c"/></model>{}</world></sdf>'


def test_normalizes_only_blank_spacing_and_runtime_prefix():
    a = ET.fromstring('<a>\n <uri>/tmp/arena_world_123/b.obj</uri>\n</a>')
    b = ET.fromstring('<a><uri>/tmp/arena_world_other/b.obj</uri></a>')
    assert normalized_xml(a) == normalized_xml(b)
    b.find('uri').text = '/tmp/arena_world_other/c.obj'
    assert normalized_xml(a) != normalized_xml(b)


def test_embedded_shape_checks_detect_vehicle_change(tmp_path):
    embedded = MODEL.replace('</model>', '<pose>1 2 0.01 0 0 0.5</pose></model>')
    (tmp_path/'runtime_vehicle.sdf').write_text('<sdf>'+MODEL+'</sdf>')
    (tmp_path/'runtime_world.sdf').write_text(WORLD.format(embedded))
    report = {'evaluation_startup': {'mode':'embedded_paused'}, 'spawn':{'x':1.,'y':2.,'yaw_rad':.5}}
    first = world_and_model(tmp_path, report)
    assert first['passed']
    (tmp_path/'runtime_world.sdf').write_text(WORLD.format(embedded.replace('<mass>2', '<mass>3')))
    assert not world_and_model(tmp_path, report)['passed']
    (tmp_path/'runtime_world.sdf').write_text(WORLD.format(''))
    assert not world_and_model(tmp_path, report)['passed']
    report['evaluation_startup']['mode'] = 'service'
    assert world_and_model(tmp_path, report)['world_without_vehicle_sha256'] == first['world_without_vehicle_sha256']


def test_missing_readiness_rejected():
    assert not readiness_evidence({})
    assert not readiness_evidence({'readiness':{}})


def sample(mode):
    return {'mode':mode, 'passed':True, 'shape':{'world_without_vehicle_sha256':'world'},
            'trace':{'spawn':{'iteration':1}, 'speed':[], 'steering':[]},
            **{k:'same' for k in ('source_sha256','meshes_sha256','profile','spawn_pose','runtime_vehicle_sha256')}}


def test_comparison_requires_full_registered_set_and_same_workload():
    runs = [sample('service'), sample('embedded_paused'), sample('embedded_paused')]
    assert compare(runs)['passed']
    assert not compare(runs[:2])['passed']
    for field in ('source_sha256', 'runtime_vehicle_sha256', 'profile', 'spawn_pose', 'meshes_sha256'):
        changed = copy.deepcopy(runs); changed[2][field] = 'changed'
        assert not compare(changed)['passed']
    changed = copy.deepcopy(runs); changed[2]['trace']['spawn']['iteration'] = 2
    assert not compare(changed)['passed']
    changed = copy.deepcopy(runs); changed[2]['passed'] = False
    assert not compare(changed)['passed']


def test_bounded_confirmation_never_claims_repetition_or_unregistered_change():
    a, b = sample('service'), sample('embedded_paused')
    a['source_sha256'] = {'helper':'before','control':'same'}
    b['source_sha256'] = {'helper':'after','control':'same'}
    c = {'approved_implementation_changes':{'helper':{'before':'before','after':'after'}}}
    result = bounded_confirmation(a,b,c)
    assert result['single_corrected_run_confirmed']
    assert not result['repeatability_verified'] and not result['command_application_determinism_verified']
    b['source_sha256']['control'] = 'changed'
    assert not bounded_confirmation(a,b,c)['single_corrected_run_confirmed']
