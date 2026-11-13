import pytest
import numpy as np

from analyze_follow_trace import agreement_distance, agreement_support, analyze
from test_follow_trace import make_trace


def test_agreement_distance_matches_geometry():
    path = np.column_stack((np.linspace(.01, .8, 36), np.zeros(36)))
    assert agreement_distance(path, path+[0., .06]) == pytest.approx(.06)
    assert agreement_distance(path[:2], path) is None


def test_distance_uses_common_domain_like_runtime():
    path = np.column_stack((np.linspace(.05, .65, 80), np.zeros(80)))
    shortened = path[path[:, 0] >= .28]+[0., .01]
    assert agreement_distance(path, shortened) == pytest.approx(.01)
    assert agreement_distance(path, path[path[:, 0] >= .5]) is None
    assert agreement_support(path, shortened)['support_sufficient']
    assert not agreement_support(path, path[path[:, 0] >= .5])['support_sufficient']
    assert agreement_support(path, path[:0])['common'] == []


def test_instrumentation_does_not_change_decision(tmp_path):
    _, events = make_trace(tmp_path)
    result = analyze(events)
    assert result['passed'] and result['calls'] == result['baseline_matched'] == 7
    assert result['instrumented_mismatches'] == []
    assert any(r['probes'] for r in result['rows'])


def test_mismatch_stops_analysis(tmp_path):
    _, events = make_trace(tmp_path)
    next(e for e in events if e['kind'] == 'call')['data']['output'][0] = 22.
    result = analyze(events)
    assert not result['passed'] and result['error'] == 'baseline_replay_mismatch'
