"""원본/OFF/ON의 새 자식에서 동일 고유 저장 계획 순서를 대조한다. 원 주행의 누락 이력은 복원하지 않는다."""
import argparse
from collections import Counter
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
TRACE = A.parent / 'final_runtime_drive_r406_v1/run1/follow_decisions.jsonl'


def encoded(value):
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(',', ':')).encode()


def write(path, value):
    with path.open('x', encoding='utf8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def frozen_inputs():
    manifest = json.loads((A / 'manifest.json').read_bytes())
    for item in manifest['files']:
        if digest(ROOT / item['path']) != item['sha256']:
            raise AssertionError('frozen input changed: ' + item['path'])


def corpus():
    """outer output은 원 plan 출력. 중복은 input/output까지 같아야 한 번만 계산한다."""
    records, seen, current_ids, discarded = [], {}, set(), []
    reason = None
    calls = duplicates = 0
    for line in TRACE.open(encoding='utf8'):
        event = json.loads(line); data = event['data']
        if event['kind'] == 'header':
            header = data
        elif event['kind'] == 'reset':
            reason = data.get('reason')
        elif event['kind'] == 'parallel_discard':
            discarded.append(data)
        elif event['kind'] == 'call':
            calls += 1
            compute = data['compute']; identity = (compute['generation'], compute['sequence'])
            assert compute['input'] == data['input']
            identity_value = encoded([data['input'], data['output']])
            if identity in seen:
                assert seen[identity] == identity_value, ('conflicting duplicate', identity)
                duplicates += 1
            else:
                seen[identity] = identity_value
                records.append(dict(trace_seq=event['seq'], generation=identity[0], sequence=identity[1],
                    reset_reason=reason, data=data['input'], recorded_output=data['output']))
                reason = None
            current = data.get('current_rebase')
            if current:
                current_ids.add((current['compute']['generation'], current['compute']['sequence']))
    assert calls == 265 and duplicates == 11 and len(records) == 254 and len(current_ids) == 153
    all_ids = set(seen) | current_ids
    assert {g for g, _ in all_ids} == {361}
    missing = sorted(set(range(1, max(s for _, s in all_ids)+1)) - {s for _, s in all_ids})
    assert missing == [74,75,78,81,84,90,91,100,103,106,109,115,116,119,122,125,
                       247,248,251,265,266,272,273,277,278,290,297,319,431,432]
    assert len([r for r in records if r['sequence'] < missing[0]]) == 37
    assert all(a['sequence'] < b['sequence'] for a, b in zip(records, records[1:]))
    ledger = dict(raw_call_rows=calls, unique_plan_calls=len(records), identical_duplicate_rows=duplicates,
        current_id_count=len(current_ids), missing_combined_sequence=missing,
        discard_records=discarded, complete_prefix_plan_count=37, complete_prefix_max_sequence=73,
        later_open_loop_plan_count=217,
        limitation='Missing calculations have no restored input/state. Later stored-order replay is not original child-state reconstruction.')
    return header, records, ledger


def worker(arm, destination):
    frozen_inputs()
    sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
    import arena_autonomy
    if arm != 'baseline':
        arena_autonomy.__path__.insert(0, str(A/'staged/arena_autonomy'))
    # 반드시 새 process에서 첫 partial import 전에 단일 파일 overlay를 지정한다.
    from arena_autonomy import partial_observation
    from arena_autonomy.follow_trace import freeze, comparable_decision
    from arena_autonomy.race_runtime_policy.evaluator import Evaluator
    header, records, ledger = corpus()
    header = copy.deepcopy(header)
    header.update(runtime_policy='cad_runtime_r377_v1', runtime_time_policy='verified_only', numeric_execution='native_v1')
    if header.get('current_corner_budget', 'off') != 'off':
        header['current_corner_environment'] = dict(use_sim_time=True, motion_timing_verified=True, backend='native2slot_v1')
    evaluator = Evaluator('plan', header)
    selected = getattr(evaluator.follow, 'normal', evaluator.follow).partial_policy
    if arm == 'candidate':
        selected.crop_edge_component_trial = True
    else:
        assert not getattr(selected, 'crop_edge_component_trial', False)
    expected_path = (A/'staged/arena_autonomy/partial_observation.py' if arm != 'baseline'
                     else ROOT/'src/arena_autonomy/arena_autonomy/partial_observation.py')
    assert Path(partial_observation.__file__).resolve() == expected_path.resolve()
    last_generation = None
    try:
        with (destination/(arm+'.jsonl')).open('x', encoding='utf8') as stream:
            for record in records:
                if record['generation'] != last_generation:
                    evaluator.reset(record['reset_reason'])
                    last_generation = record['generation']
                before = encoded(record['data'])
                output = evaluator(record['data'])
                assert encoded(record['data']) == before, 'mutated saved input'
                normal = getattr(evaluator.follow, 'normal', evaluator.follow)
                policy = normal.partial_policy
                names = ('previous', 'previous_distinct', 'last_source', 'clear_sources',
                         'latched', 'latch_reason', 'latch_time', 'point_times', 'point_ids',
                         'anchor_latch_time', 'anchor_threat_summary', 'structure_previous',
                         'structure_hidden', 'structure_last_source')
                state = freeze({name: getattr(policy, name) for name in names})
                state['anchor_probe'] = freeze(vars(policy.anchor_probe))
                row = dict(trace_seq=record['trace_seq'], generation=record['generation'], sequence=record['sequence'],
                    reference_s=record['data']['reference_s'], output=output,
                    comparable=comparable_decision(output), state=state,
                    recorded_comparable=comparable_decision(record['recorded_output']))
                stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n'); stream.flush()
    finally:
        evaluator.close()
    write(destination/(arm+'_receipt.json'), dict(arm=arm, ledger=ledger, selected_header=header,
        partial_file=str(expected_path.relative_to(ROOT)).replace('\\','/'), partial_sha256=digest(expected_path),
        native_modules={name: dict(path=str(Path(module.__file__).resolve()), sha256=digest(Path(module.__file__)))
                        for name, module in sys.modules.items() if name.startswith('arena_autonomy')
                        and getattr(module, '__file__', None) and Path(module.__file__).suffix in ('.so','.pyd')}))
    frozen_inputs()


def normal_reason(output):
    details = output[2]
    return details.get('normal', details).get('reason')


def compare_off(destination):
    before = [json.loads(x) for x in (destination/'baseline.jsonl').open(encoding='utf8')]
    off = [json.loads(x) for x in (destination/'off.jsonl').open(encoding='utf8')]
    assert len(before) == len(off) == 254
    mismatches = [b['trace_seq'] for b, a in zip(before, off)
        if (b['generation'], b['sequence'], b['trace_seq'], b['comparable'], b['state'])
        != (a['generation'], a['sequence'], a['trace_seq'], a['comparable'], a['state'])]
    write(destination/'off_report.json', dict(records=254, mismatches=mismatches, output_and_state_exact=not mismatches))
    assert not mismatches, 'OFF changed original output/state; stop before ON'


def compare(destination):
    before = [json.loads(x) for x in (destination/'baseline.jsonl').open(encoding='utf8')]
    after = [json.loads(x) for x in (destination/'candidate.jsonl').open(encoding='utf8')]
    geometry = {r['trace_seq']: r for r in map(json.loads, (A.parent/'front_crop_boundary_r410_v1/analysis_v1/r406_rows.jsonl').open(encoding='utf8'))}
    assert len(before) == len(after) == 254
    comparisons = []; seen_target = False
    for b, a in zip(before, after):
        assert (b['generation'], b['sequence'], b['trace_seq']) == (a['generation'], a['sequence'], a['trace_seq'])
        edges = geometry[b['trace_seq']]['edges']
        detail = a['output'][2].get('normal', a['output'][2])
        component = detail.get('partial_observation', {}).get('crop_edge_component')
        direct = component is not None
        seen_target |= direct
        output_same = b['comparable'] == a['comparable']; state_same = b['state'] == a['state']
        row = dict(trace_seq=b['trace_seq'], sequence=b['sequence'], reference_s=b['reference_s'],
            direct_component=direct, original_flank_only=bool(edges) and all(e['candidate_flank_only'] for e in edges),
            outside_continuation=bool(edges) and any(e['outside_unknown_count'] for e in edges), component=component,
            output_same=output_same, state_same=state_same, before_first_target=not seen_target,
            baseline_original_same=b['comparable'] == b['recorded_comparable'],
            baseline_speed_mps=b['output'][0], candidate_speed_mps=a['output'][0],
            baseline_reason=normal_reason(b['output']), candidate_reason=normal_reason(a['output']))
        comparisons.append(row)
    prefix = [r for r in comparisons if r['sequence'] <= 73]
    early = [r for r in comparisons if r['before_first_target']]
    focus = [r for r in comparisons if r['reference_s'] >= 25]
    unexpected_early = [r['trace_seq'] for r in early if not (r['output_same'] and r['state_same'])]
    report = dict(scope='same recorded unique input order in two new policy instances; no live/reconstructed trajectory claim',
        records=254, direct_component=sum(r['direct_component'] for r in comparisons),
        prefix_original_mismatch=[r['trace_seq'] for r in prefix if not r['baseline_original_same']],
        later_original_mismatch=sum(not r['baseline_original_same'] for r in comparisons[37:]),
        unexpected_difference_before_first_target=unexpected_early,
        changed_outputs=sum(not r['output_same'] for r in comparisons),
        changed_base_partial_states=sum(not r['state_same'] for r in comparisons),
        outside_continuation_output_changes=[r['trace_seq'] for r in comparisons if r['outside_continuation'] and not r['output_same']],
        after25=dict(records=len(focus), direct_component=sum(r['direct_component'] for r in focus),
            baseline_positive=sum(r['baseline_speed_mps']>0 for r in focus),
            candidate_positive=sum(r['candidate_speed_mps']>0 for r in focus),
            baseline_reasons=dict(Counter(r['baseline_reason'] for r in focus)),
            candidate_reasons=dict(Counter(r['candidate_reason'] for r in focus))),
        state_scope='Base seven fields plus timed acquisition maps, full anchor state/summary and structure memory; not entire planner equivalence.',
        comparison_rule='All decision fields exact, except existing comparable_decision two diagnostic processing_ms fields.',
        acceptance='No early unexpected difference; prefix oracle mismatch blocks interpretation until explained. Later original mismatch is disclosed, not tuned away.')
    with (destination/'comparison_rows.jsonl').open('x', encoding='utf8') as stream:
        for row in comparisons:stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
    write(destination/'report.json', report)
    assert not unexpected_early, 'unexpected difference before any eligible boundary'
    assert not report['prefix_original_mismatch'], 'complete-prefix oracle mismatch; inspect without altering thresholds'
    print(json.dumps(report, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--worker', choices=('baseline','off','candidate')); args = parser.parse_args()
    destination = args.output.resolve()
    assert destination.is_relative_to(A)
    if args.worker:
        worker(args.worker, destination); return
    frozen_inputs()
    assert not destination.exists(), 'preserve earlier outputs; choose a new directory'
    destination.mkdir()
    _, _, ledger = corpus(); write(destination/'input_ledger.json', ledger)
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONHASHSEED='0')
    environment['PYTHONPATH'] = os.pathsep.join(str(ROOT/p) for p in ('src/arena_autonomy','src/arena_vehicle_interface'))
    commands = [('tests', [sys.executable,'-B','-m','pytest','-q','-s','-p','no:cacheprovider',str(A/'test_edge_component.py')], 300)]
    commands += [(arm,[sys.executable,'-B',str(Path(__file__).resolve()),'--output',str(destination),'--worker',arm],600)
                 for arm in ('baseline','off','candidate')]
    for name, command, timeout in commands:
        with (destination/(name+'.log')).open('x', encoding='utf8') as stream:
            result = subprocess.run(command, cwd=ROOT, env=environment, stdout=stream, stderr=subprocess.STDOUT, timeout=timeout)
        frozen_inputs()
        assert result.returncode == 0, (name, result.returncode)
        if name == 'off':
            compare_off(destination)
    compare(destination)
    frozen_inputs()


if __name__ == '__main__':main()
