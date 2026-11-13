import copy
import json
import pytest
from audit_saved import compare_headers, parse_sectors, summarize


def header():
    data = {'sectors_deg': [[168.0, 3.0], [-168.0, 3.1]],
            'range_limits_m': [[.15, .18], [.15, .19]]}
    return {'follow_config': {'known_structure_sectors_json': json.dumps(data), 'width': .15}, 'other': True}


def test_whitespace_only_is_equivalent_but_not_exact():
    a = header(); b = copy.deepcopy(a)
    b['follow_config']['known_structure_sectors_json'] = json.dumps(
        json.loads(b['follow_config']['known_structure_sectors_json']), separators=(',', ':'))
    assert compare_headers(a, b) == dict(equivalent=True, exact=False, error=None,
        normalized_field='follow_config.known_structure_sectors_json', values_types_and_order_preserved=True)


@pytest.mark.parametrize('change', ['number', 'order', 'type', 'other', 'missing'])
def test_actual_changes_fail(change):
    a = header(); b = copy.deepcopy(a)
    data = json.loads(b['follow_config']['known_structure_sectors_json'])
    if change == 'number': data['sectors_deg'][0][1] += .01
    elif change == 'order': data['sectors_deg'].reverse()
    elif change == 'type': data['sectors_deg'][0][0] = 168
    elif change == 'other': b['other'] = 1
    elif change == 'missing': del b['follow_config']['width']
    b['follow_config']['known_structure_sectors_json'] = json.dumps(data)
    assert not compare_headers(a, b)['equivalent']


@pytest.mark.parametrize('text', [
    '{', '{"sectors_deg":[],"sectors_deg":[],"range_limits_m":[]}',
    '{"sectors_deg":[[NaN,1]],"range_limits_m":[[0,1]]}',
    '{"sectors_deg":[[true,1]],"range_limits_m":[[0,1]]}',
    '{"sectors_deg":[[1,2]],"range_limits_m":[]}', '{"unexpected":[]}'])
def test_invalid_json_or_schema_rejected(text):
    with pytest.raises(ValueError): parse_sectors(text)


@pytest.mark.parametrize('fault', ['config', 'footer', 'capture', 'factory'])
def test_invalid_evidence_cannot_claim_publication_valid(fault):
    expected = header(); actual = copy.deepcopy(expected)
    factory = dict(policy='cad_runtime_r377_v1', current_corner_budget='r396_v1',
                   motion_time_policy='verified_only', motion_timing_verified=True)
    rows = [dict(kind='header', data=actual), dict(kind='runtime_factory_policy', data=factory),
            dict(kind='decision_submission_policy', data={'policy': 'runtime_dual_latest_v1'}),
            dict(kind='footer', data={'capture_errors': 0})]
    if fault == 'config': actual['other'] = False
    elif fault == 'footer': rows.pop()
    elif fault == 'capture': rows[-1]['data']['capture_errors'] = 1
    elif fault == 'factory': factory['current_corner_budget'] = 'off'
    out = summarize(rows, expected, dict(candidate_parent_publications=1, header_exact=False))
    assert not out['parent_publication_evidence_valid']
    assert not out['driving_improvement_demonstrated']
