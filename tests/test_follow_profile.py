import copy
import gc

import pytest

from profile_follow_trace import profile, profile_source, read_events, digest
from test_follow_trace import make_trace


def test_profile_preserves_all_outputs_including_error_reset(tmp_path):
    _, events = make_trace(tmp_path)
    result = profile(events, repeats=2, warmups=1)
    assert result['passed'] and result['baseline_matched'] == 7
    assert result['timed_calls'] == 14
    assert result['output_mismatches'] == []
    assert any(r['failed_stage'] == 'projection' for r in result['rows'])
    assert not result['online_deadline_verified']
    for row in result['rows']:
        assert row['total_ms'] == pytest.approx(row['restore_ms']+row['projection_ms']+row['decision_ms'])


def test_corrupt_capture_or_different_output_is_not_benchmarked(tmp_path):
    _, events = make_trace(tmp_path)
    with pytest.raises(ValueError, match='footer'):
        profile(events[:-1])
    changed = copy.deepcopy(events)
    next(e for e in changed if e['kind'] == 'call')['data']['output'][0] += .5
    with pytest.raises(ValueError, match='outputs'):
        profile(changed)


@pytest.mark.parametrize('repeats,warmups', [(0, 1), (11, 1), (1, 4), (True, 1), (1, -1)])
def test_profile_run_count_bounded(repeats, warmups):
    with pytest.raises(ValueError, match='bounded'):
        profile([], repeats=repeats, warmups=warmups)


def test_timing_does_not_mutate_recorded_input(tmp_path):
    _, events = make_trace(tmp_path)
    before = copy.deepcopy(events)
    profile(events, repeats=1, warmups=0)
    assert events == before


def test_gc_settings_and_callbacks_restored_on_success_and_failure(tmp_path):
    _, events = make_trace(tmp_path)
    before = list(gc.callbacks), gc.get_threshold(), gc.isenabled()
    result = profile(events, repeats=1, warmups=0)
    assert (list(gc.callbacks), gc.get_threshold(), gc.isenabled()) == before
    assert all(r['thread_cpu_ms'] >= 0 and r['gc_elapsed_ms'] >= 0 for r in result['rows'])
    assert 'no_gc_observed' in result['groups']
    with pytest.raises(ValueError):
        profile(events[:-1])
    assert (list(gc.callbacks), gc.get_threshold(), gc.isenabled()) == before


def test_streaming_and_loaded_match_outputs_with_same_call_count(tmp_path):
    path, events = make_trace(tmp_path)
    loaded = profile(events, repeats=1, warmups=0)
    streamed = profile_source(lambda: read_events(path), repeats=1, warmups=0)
    assert streamed['passed'] and streamed['timed_calls'] == loaded['timed_calls']
    assert [(r['seq'], r['reason'], r['selection']) for r in streamed['rows']] == [
        (r['seq'], r['reason'], r['selection']) for r in loaded['rows']]


def test_portable_hash_does_not_require_python_311(tmp_path, monkeypatch):
    import hashlib
    monkeypatch.delattr(hashlib, 'file_digest', raising=False)
    path = tmp_path/'testdata'
    path.write_bytes(b'g1')
    assert digest(path) == hashlib.sha256(b'g1').hexdigest()
