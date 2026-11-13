"""복수 actor 시험기의 자료 분리. 실제 주행 성공을 주장하지 않는 단위 회귀."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import traffic_actors as ta


def fixture():
    config = json.loads((ROOT/'config/tests/g1_traffic_v1.json').read_text(encoding='utf-8'))
    case = deepcopy(config['cases'][0])
    case['additional_actors'] = [{'model_name': 'eval_side', 'role': 'side',
        'start_center_gap_m': -.8, 'lateral_offset_m': .15, 'schedule': [],
        'minimum_body_gap_m': .15, 'gap_basis': 'unit fixture'}]
    return config, case


def test_legacy_one_actor_contract_unchanged():
    config, _ = fixture()
    actor, = ta.actor_specs(config, config['cases'][0])
    assert actor['model_name'] == 'eval_g1b_actor'
    assert actor['contact_topic'] == '/eval/g1b_actor/contact'
    assert actor['schedule'] == []


def test_extra_does_not_mutate_primary_config():
    config, case = fixture()
    before = deepcopy(config)
    actors = ta.actor_specs(config, case)
    assert len(actors) == 2 and actors[1]['contact_topic'] != actors[0]['contact_topic']
    assert actors[1]['start_center_gap_m'] == -.8
    assert config == before


@pytest.mark.parametrize('change', [
    {'model_name': 'eval_g1b_actor'}, {'model_name': 'bad/name'}, {'role': 'wall'},
    {'width_m': 0}, {'bottom_m': -1}, {'command_hz': float('nan')},
    {'start_center_gap_m': float('inf')}, {'lateral_offset_m': .6},
    {'schedule': [{'from_s': 0, 'to_s': 1, 'speed_mps': .2}]}])
def test_bad_actors_rejected(change):
    config, case = fixture()
    case['additional_actors'][0].update(change)
    with pytest.raises(ValueError):
        ta.actor_specs(config, case)


def test_separate_pose_stream_and_duplicate_timestamp_diagnostics():
    specs = ta.actor_specs(*fixture())
    records = ta.ActorRecords(specs)
    for spec in specs:
        assert records.pose(spec['model_name'], 'it_arena_track', (1., 0., 0., 0.))
    assert not records.pose('eval_side', 'it_arena_track', (1., 0., 0., 0.))
    assert records.rejected_times == {'eval_g1b_actor': 0, 'eval_side': 1}
    assert not records.pose('eval_unknown', 'it_arena_track', (2., 0., 0., 0.))
    assert not records.pose('eval_side', 'wrong_frame', (2., 0., 0., 0.))
    assert len(records.poses['eval_side']) == 1
    with pytest.raises(ValueError):
        records.pose('eval_side', 'it_arena_track', (2., float('nan'), 0., 0.))


def test_contact_pair_dedup_keeps_source_evidence():
    specs = ta.actor_specs(*fixture())
    pair = ['eval_side::body::collision', 'eval_g1b_actor::body::collision']
    rows = [{'actor': a['model_name'], 'topic': a['contact_topic'], 't': 1., 'pairs': [pair]} for a in specs]
    result = ta.contact_events(rows, specs)
    assert result['valid'] and result['duplicate_pair_reports'] == 1 and len(result['events']) == 1
    assert len(result['events'][0]['sources']) == 2 and result['ego_events'] == 0
    rows.append({**rows[1], 't': 1.01, 'pairs': [['arena_car::chassis::collision', pair[0]]]})
    result = ta.contact_events(rows, specs)
    assert result['ego_events'] == 1 and len(result['events']) == 2


def test_misrouted_contact_not_counted_as_valid():
    specs = ta.actor_specs(*fixture())
    for row in ({'actor': 'eval_side', 'topic': specs[0]['contact_topic'], 't': 1., 'pairs': []},
                {'actor': 'eval_side', 'topic': specs[1]['contact_topic'], 't': 1.,
                 'pairs': [['arena_car::c', 'wall::c']]},
                {'actor': 'eval_side', 'topic': specs[1]['contact_topic'], 't': float('nan'), 'pairs': []}):
        assert not ta.contact_events([row], specs)['valid']
    empty = ta.contact_events([], specs)
    assert empty['valid'] and 'not no-contact proof' in empty['scope']


def test_actor_missing_cannot_borrow_other_actor_evidence():
    config, case = fixture()
    spec = ta.actor_specs(config, case)[1]
    rows = [(i*.01, 0., 0., 0.) for i in range(101)]
    runtime = '<sdf><world><model name="eval_side"><static>true</static></model></world></sdf>'
    def check(poses, world=runtime):
        return ta.audit_actor(spec, poses, [], 0., 1., lambda t: (0., 0.), world, config['acceptance']['harness'])
    assert check(rows)['valid']
    assert not check([])['valid']
    assert not check(rows[:20]+rows[50:])['valid']
    assert not check(rows, '<sdf/>')['valid']
    assert not check([(t, x+1, y, a) for t, x, y, a in rows])['valid']


def test_compose_has_independent_models_topics_and_one_contact_plugin():
    specs = ta.actor_specs(*fixture())
    def single(config, pose):
        return ('<sdf><plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>'
                f'<model name="{config["actor"]["model_name"]}"><link><sensor><contact>'
                '<topic>old</topic></contact></sensor></link></model></sdf>')
    root = ET.fromstring(ta.compose_sdf(specs, [(0., 0., 0.)]*2, single))
    assert len(root.findall('plugin')) == 1 and len(root.findall('model')) == 2
    assert [m.findtext('link/sensor/contact/topic') for m in root.findall('model')] == [s['contact_topic'] for s in specs]


def test_preregistered_overlay_requires_matching_case_and_keeps_original():
    config, _ = fixture()
    case = config['cases'][0]
    assert ta.overlay_case(case, None) is case
    added = ta.overlay_case(case, ROOT/'config/tests/e3_two_actors_harness_v1.json')
    assert len(ta.actor_specs(config, added)) == 2 and 'additional_actors' not in case
    with pytest.raises(ValueError):
        ta.overlay_case(config['cases'][1], ROOT/'config/tests/e3_two_actors_harness_v1.json')
