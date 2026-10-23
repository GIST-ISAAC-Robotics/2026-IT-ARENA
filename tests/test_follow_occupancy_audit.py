import copy
from dataclasses import asdict
import math

import pytest

from arena_autonomy.local_follow import FollowConfig
from audit_follow_occupancy import audit_events


def records(mode='per_ray_candidate'):
    cfg = asdict(FollowConfig(occupancy_timing_mode=mode))
    meta = dict(occupied_points=2, unfiltered_occupancy=True, reason='local_follow',
                occupancy_padding_m=.155)
    if mode == 'per_ray_candidate':
        meta.update(occupancy_timing_mode=mode, occupancy_padding_points=2,
                    occupancy_padding_min_m=.145, occupancy_padding_max_m=.155)
    data = dict(reference_s=1.2, epoch=0, rear_half_angle_deg=30., scan=dict(
        sec=1, nanosec=0, time_increment=.02, angle_min=0., angle_increment=math.pi/2,
        range_min=.05, range_max=12., ranges=[1., 2., 3., {'nonfinite':'inf'}]))
    return [dict(seq=0, kind='header', data=dict(schema=1, scope='g1_decision_stage_only', follow_config=cfg)),
            dict(seq=1, kind='call', data=dict(input=data, output=[.2, 0., meta])),
            dict(seq=2, kind='footer', data=dict(capture_errors=0, failed=0, dropped=0, pending=0,
                                               worker_alive=False, written=2, accepted=2))]


@pytest.mark.parametrize('mode', ['oldest', 'per_ray_candidate'])
def test_original_acquisition_and_mask_count_independently_reconstructed(mode):
    events = records(mode)
    before = copy.deepcopy(events)
    result = audit_events(events, mode)
    assert result['passed'] and result['evaluated_calls'] == 1 and result['rows'][0]['returns'] == 2
    assert result['maximum_first_ray_age_s'] == pytest.approx(.2)
    assert events == before and not result['driving_authorized'] and not result['output_replay_verified']


@pytest.mark.parametrize('field,value', [('occupancy_padding_min_m', .055), ('occupancy_padding_max_m', .06),
    ('occupancy_padding_m', .055), ('occupancy_padding_points', 3), ('occupied_points', 3),
    ('unfiltered_occupancy', False), ('occupancy_timing_mode', 'oldest')])
def test_freshening_or_dropping_or_wrong_mode_not_accepted(field, value):
    events = records()
    events[1]['data']['output'][2][field] = value
    result = audit_events(events, 'per_ray_candidate')
    assert not result['passed'] and result['errors']


@pytest.mark.parametrize('fault', ['no_footer', 'gap', 'dropped', 'future', 'stale', 'mode'])
def test_invalid_capture_is_not_success(fault):
    events = records()
    if fault == 'no_footer': events.pop()
    elif fault == 'gap': events[1]['seq'] = 8
    elif fault == 'dropped': events[-1]['data']['dropped'] = 1
    elif fault == 'future': events[1]['data']['input']['reference_s'] = 1.03
    elif fault == 'stale': events[1]['data']['input']['reference_s'] = 1.31
    else: events[0]['data']['follow_config']['occupancy_timing_mode'] = 'oldest'
    with pytest.raises(ValueError): audit_events(events, 'per_ray_candidate')


def test_no_evaluated_occupancy_not_vacuous_success():
    events = records()
    events[1]['data']['output'] = [0., 0., {'reason':'follow_input_stop'}]
    result = audit_events(events, 'per_ray_candidate')
    assert not result['passed'] and result['evaluated_calls'] == 0
    assert result['skipped_calls'] == {'follow_input_stop':1}


def test_aging_same_scan_increases_padding_without_new_observation():
    events = records()
    later = copy.deepcopy(events[1])
    later['seq'] = 2
    later['data']['input']['reference_s'] += .04
    for key in ('occupancy_padding_m', 'occupancy_padding_min_m', 'occupancy_padding_max_m'):
        later['data']['output'][2][key] += .02
    events.insert(2, later)
    events[-1]['seq'] = 3
    events[-1]['data'].update(written=3, accepted=3)
    result = audit_events(events, 'per_ray_candidate')
    assert result['passed'] and result['evaluated_calls'] == 2 and result['unique_evaluated_scans'] == 1


def test_cli_captures_sources_before_work_and_detects_mid_audit_change(tmp_path, monkeypatch):
    import json
    import sys
    import zipfile
    import audit_follow_occupancy as audit
    source = tmp_path/'scripts/audit_follow_occupancy.py'
    dependency = tmp_path/'src/arena_autonomy/arena_autonomy/follow_trace.py'
    for path in (source, dependency):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('original source\n')
    capture, output = tmp_path/'capture.jsonl', tmp_path/'audit.json'
    capture.write_text('\n'.join(json.dumps(row) for row in records())+'\n')
    monkeypatch.setattr(audit, 'ROOT', tmp_path)
    monkeypatch.setattr(audit, '__file__', str(source))
    monkeypatch.setattr(sys, 'argv', ['audit', str(capture), '--mode', 'per_ray_candidate', '--output', str(output)])
    original = audit.audit_events
    def changed(events, mode):
        dependency.write_text('changed during audit\n')
        return original(events, mode)
    monkeypatch.setattr(audit, 'audit_events', changed)
    assert audit.main() == 2
    report = json.loads(output.read_text())
    assert not report['passed'] and not report['source_unchanged'] and report['input_unchanged']
    with zipfile.ZipFile(output.with_suffix('.source.zip')) as bundle:
        assert bundle.read('src/arena_autonomy/arena_autonomy/follow_trace.py') == b'original source\n'
