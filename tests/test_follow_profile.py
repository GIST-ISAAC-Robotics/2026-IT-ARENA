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


def test_snapshot_covers_transitive_internal_imports_including_per_ray_timing():
    import ast
    from profile_follow_trace import ROOT, SNAPSHOT_MODULES
    manifest = {f'{package}.{name}' for package, names in SNAPSHOT_MODULES.items() for name in names}
    assert 'arena_autonomy.occupancy_timing' in manifest
    for module in manifest:
        package, name = module.split('.')
        tree = ast.parse((ROOT/'src'/package/package/f'{name}.py').read_text(encoding='utf-8'))
        imports = [node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) and node.module]
        imports += [alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names]
        required = {name for name in imports if name.startswith(('arena_autonomy.', 'arena_vehicle_interface.'))}
        assert required <= manifest, (module, sorted(required-manifest))


def test_partial_candidate_profile_uses_raw_scan_and_compares_decisions(tmp_path, monkeypatch):
    import json
    from arena_autonomy.follow_trace import FollowTrace
    from arena_autonomy.local_follow import FollowConfig, LocalFollow
    from test_g1_partial_policy import corridor, call
    from test_local_follow import history
    cfg = FollowConfig(partial_observation_policy='bounded_local_v4', parallel_hold_trial=True,
                       pass_release_trial=True, boundary_evidence_mode='corner_distance_candidate',
                       occupancy_timing_mode='per_ray_candidate')
    original = LocalFollow.command
    calls = []

    def measured(self, *args, **kwargs):
        assert kwargs.get('raw_scan') is not None
        result = original(self, *args, **kwargs)
        # 비용만 매 호출 달라도 비교가 되어야 하며 명령/진단은 계속 대조한다.
        calls.append(1)
        result[2].setdefault('partial_observation', {})['surface_correspondence'] = {
            'processing_ms': float(len(calls)), 'accepted': False}
        return result

    monkeypatch.setattr(LocalFollow, 'command', measured)
    h = history(); follow = LocalFollow(cfg)
    path = tmp_path/'partial_trace.jsonl'
    trace = FollowTrace(path, h.config, cfg)
    for first in (.3, .4, .5):
        scan = corridor(first)
        out = call(follow, scan, h)
        trace.record_call(scan, h, first+.1, 0, 0., 'left', 0., .02, True, 30., out)
    trace.close()
    events = [json.loads(line) for line in path.read_text().splitlines()]
    result = profile(events, repeats=1, warmups=0)
    assert result['passed'] and result['timed_calls'] == 3
    changed = copy.deepcopy(events)
    next(e for e in changed if e['kind'] == 'call')['data']['output'][0] += .01
    with pytest.raises(ValueError, match='outputs'):
        profile(changed, repeats=1, warmups=0)
