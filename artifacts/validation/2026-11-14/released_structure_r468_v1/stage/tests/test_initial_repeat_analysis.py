import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_initial_repeat import startup_pair, stage_differences, application_rows


def sample(name):
    return {'name':name, 'mode':'embedded_paused', 'passed':True,
            **{k:'same' for k in ('source_sha256','meshes_sha256','profile','spawn_pose','runtime_vehicle_sha256')},
            'command_start_ns':2_010_000_000, 'shape':{'world_without_vehicle_sha256':'same'},
            'startup':{'first_update':{'speed':0.,'steering':0.}},
            'trace':{'spawn':{'iteration':1,'sim_ns':1_000_000,'dt_ns':1_000_000}}}


def test_startup_pair_does_not_require_matching_dynamic_results():
    a,b = sample('a'),sample('b')
    a['peak_body_speed_mps'],b['peak_body_speed_mps'] = 1.,2.
    assert startup_pair(a,b)['bounded_initialization_repeat_confirmed']
    assert not startup_pair(a,a)['bounded_initialization_repeat_confirmed']


@pytest.mark.parametrize('field', ['source_sha256','meshes_sha256','profile','spawn_pose',
                                  'runtime_vehicle_sha256','command_start_ns'])
def test_changed_condition_rejects_repeat(field):
    a,b = sample('a'),sample('b')
    b[field] = 'changed'
    assert not startup_pair(a,b)['bounded_initialization_repeat_confirmed']


@pytest.mark.parametrize('mutation', ['failed','service','world','first_step','nonzero_command','missing_command'])
def test_invalid_or_different_start_rejected(mutation):
    a,b = sample('a'),sample('b')
    if mutation == 'failed': b['passed'] = False
    if mutation == 'service': b['mode'] = 'service'
    if mutation == 'world': b['shape']['world_without_vehicle_sha256'] = 'other'
    if mutation == 'first_step': b['trace']['spawn']['iteration'] = 2
    if mutation == 'nonzero_command': b['startup']['first_update']['speed'] = .1
    if mutation == 'missing_command': b.pop('startup')
    assert not startup_pair(a,b)['bounded_initialization_repeat_confirmed']


def delivery():
    return {c:{'events':2, 'stages_rel_start_ns':{s:[10,20] for s in
            ('request','request_observed','safe','mcu','output')}} for c in ('speed','steering')}


def test_stage_missing_cannot_be_equal_and_sign_preserved():
    a,b = delivery(),delivery()
    b['speed']['stages_rel_start_ns']['output'] = [9,22]
    b['speed']['stages_rel_start_ns']['safe'] = None
    result = stage_differences(a,b)
    assert result['speed']['output']['delta_b_minus_a_ns'] == [-1,2]
    assert result['speed']['output']['differing'] == 2
    assert not result['speed']['safe']['comparable']
    assert not result['steering']['output']['comparable']
    a['speed']['events'] = 3
    assert not stage_differences(a,b)['speed']['mcu']['comparable']


def test_application_rows_preserve_values_timing_and_unmatched():
    event = {'to':.5,'apply_rel_ns':10,'rx_after_rel_ns':9,'steps_after_rx':1}
    a = {c:[event.copy()] for c in ('speed','steering')}
    b = copy.deepcopy(a)
    b['speed'][0].update(apply_rel_ns=11,rx_after_rel_ns=10)
    b['steering'][0]['to'] = .6
    out = application_rows(a,b)
    assert out['speed']['transitions'][0]['apply_delta_ns'] == 1
    assert out['speed']['transitions'][0]['rx_after_delta_ns'] == 1
    assert out['speed']['transitions'][0]['steps_after_rx'] == [1,1]
    assert not out['steering']['comparable']
