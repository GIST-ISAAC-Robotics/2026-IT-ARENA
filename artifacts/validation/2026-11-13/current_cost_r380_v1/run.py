"""r375 단독 직렬 비용 대표: 기존 572저장 입력+17반례 순서/판정을 재사용한다."""
import argparse
import collections
import json
import math
from pathlib import Path
import platform
import statistics
import sys
import time

from support import A, ROOT, EXPECTED, digest, load_comparison, verify_manifest, check_imports


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered: return None
    index = (len(ordered)-1)*fraction
    lower = math.floor(index); upper = math.ceil(index)
    return ordered[lower]+(ordered[upper]-ordered[lower])*(index-lower)


def summary(rows):
    values = [row['rebase_wall_ms'] for row in rows]
    assert all(math.isfinite(v) and v >= 0 for v in values)
    return dict(count=len(values), p50_ms=percentile(values, .5), p95_ms=percentile(values, .95),
        max_ms=max(values) if values else None,
        above_100ms=sum(v > 100 for v in values), above_200ms=sum(v > 200 for v in values))


def records_in_original_order(compare):
    rows = compare.load_trace(compare.TRACE, 'r371')
    assert len(rows) == 217
    rows.extend(compare.load_trace(A.parent/'released_crawl_drive_r364_v1/run1/follow_decisions.jsonl', 'r364_non_target'))
    hold = A.parent/'current_input_jetson_r296_v1'
    headers = json.loads((hold/'headers.json').read_bytes())
    for selected in json.loads((hold/'selected_inputs.json').read_bytes()):
        row = selected['input']; label = selected['source']
        rows.append(dict(label='holdout_'+label, header=headers[label], output=row['output'],
            submitted=compare.plan_reference(row['input']), current=row['current_rebase']['input']))
    assert len(rows) == 572
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-machine', choices=('aarch64', 'any'), default='aarch64')
    args = parser.parse_args()
    if args.require_machine != 'any':
        assert platform.machine() == args.require_machine, 'Jetson arm64 run required'
    output = args.output.resolve()
    assert output.is_relative_to(A) and not output.exists(), 'fresh output under r380 required'
    output.mkdir()
    manifest = verify_manifest()
    compare = load_comparison()
    imports = check_imports(compare, 'native_v1')
    expected = json.loads(EXPECTED.read_bytes())
    assert expected['passed'] and expected['original_current_reproduced'] == 217
    records = records_in_original_order(compare)
    raw = []; checked_rows = []; negatives = []; final = None
    context = dict(phase='stored', index=0)
    original_evaluate = compare.evaluate

    def evaluate(record):
        # 기존 r375 compare의 순서 그대로: 현재 장면 복원 → 기준 r370 → r375.
        # 기준 출력 비교/복원/JSON/파일 I/O는 r375 rebase 비용에 포함하지 않는다.
        cfg = compare.FollowConfig(**record['header']['follow_config'])
        motion = compare.MotionConfig(**record['header']['motion_config'])
        current = record['current']
        info, scan, history = compare.restore_call(current, motion)
        scene = compare.motion_scene(scan, history, info['reference_s'], timing_verified=info['timing_verified'],
            epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'])
        call = (record['output'], record['submitted'], current, scene, history, cfg)
        before = compare.freeze(compare.policy.prior.rebase(*call, require_current_region=True))
        began = time.perf_counter_ns()
        result = compare.policy.rebase(*call, require_current_region=True)
        elapsed = (time.perf_counter_ns()-began)/1e6
        after = compare.freeze(result)
        normal = record['output'][2].get('normal') or {}
        diagnostic = after.get(compare.policy.DIAGNOSTIC) or {}
        raw.append(dict(phase=context['phase'], index=context['index'], label=record['label'],
            reference_s=current['reference_s'], path_source_s=normal.get('path_source_s'),
            selector_mode=normal.get('selector_mode'), selector_choice=normal.get('selector_choice'),
            applied=diagnostic.get('applied') is True, rebase_wall_ms=elapsed,
            before=before, after=after))
        return before, after

    compare.evaluate = evaluate
    began_run = time.perf_counter_ns()
    error = None
    try:
        for index, record in enumerate(records):
            context['index'] = index
            row, after = compare.compare_record(index, record)
            assert compare.equal_output(row, expected['rows'][index]), (index, 'r375 stored result mismatch')
            if record['label'] != 'r371': assert not row['applied'], (index, 'holdout changed')
            checked_rows.append(row)
            if index == 216: final = after
        assert final and final.get(compare.policy.DIAGNOSTIC, {}).get('applied')
        context.update(phase='counterexample', index=0)
        negatives = compare.counterexamples(records[216], final)
        assert compare.equal_output(negatives, expected['counterexamples']), 'counterexample result mismatch'
        assert len(raw) == 589 and len(negatives) == 17
        for index, (row, negative) in enumerate(zip(raw[572:], negatives)):
            row['index'] = index; row['counterexample'] = negative['name']
        assert sum(row['applied'] for row in raw[:217]) == 93
        verify_manifest()
    except Exception as exc:
        error = repr(exc)
        raise
    finally:
        compare.evaluate = original_evaluate
        with (output/'timing_rows.jsonl').open('x', encoding='utf8') as stream:
            for row in raw: stream.write(json.dumps(compare.freeze(row), ensure_ascii=False)+'\n')
        stored = [r for r in raw if r['phase'] == 'stored']
        primary = [r for r in stored if r['label'] == 'r371']
        groups = {'stored_all': summary(stored), 'r371_all': summary(primary),
            'r371_applied': summary([r for r in primary if r['applied']]),
            'r371_unapplied': summary([r for r in primary if not r['applied']]),
            'holdout_all': summary([r for r in stored if r['label'] != 'r371']),
            'counterexamples_all': summary([r for r in raw if r['phase'] == 'counterexample'])}
        for label in sorted({r['label'] for r in stored}):
            for mode in sorted({str(r['selector_mode']) for r in stored if r['label'] == label}):
                groups[label+':'+mode] = summary([r for r in stored if r['label'] == label and str(r['selector_mode']) == mode])
        report = dict(passed=error is None, error=error, groups=groups,
            completed_stored=len(checked_rows), completed_counterexamples=len(negatives),
            original_reproduced=217 if len(checked_rows) >= 217 else None,
            reference_comparison_sha256=digest(EXPECTED), imports=imports,
            input_manifest_sha256=digest(A/'source_manifest.json'),
            source_input_files=len(manifest['files']),
            total_wall_s=(time.perf_counter_ns()-began_run)/1e9,
            machine=platform.machine(), python=sys.version, clock='time.perf_counter_ns',
            measurement='top-level r375 rebase only; its internal r370/repeated checks remain included',
            ordering='one original-order serial pass; untimed baseline immediately precedes candidate',
            warmup='no discarded warmup; first call retained; same-process cache effects remain',
            command_expiry_evaluated=False, publication_evaluated=False, deadline_permission=False,
            threshold_scope='100ms는 검토 예산 참고, 200ms도 계산 초과 수만. 안전 허가/명령 만료 통과 아님.',
            devices_opened=False, source_timestamps_renewed=False, default_changed=False,
            scope='저장 입력 직렬 계산 비용 대표. ROS/센서 동시부하/계획 병렬/명령 전달/실차 검증이 아니다.')
        with (output/'report.json').open('x', encoding='utf8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2); stream.write('\n')
        print(json.dumps(dict(passed=report['passed'], groups=groups, error=error), ensure_ascii=False))


if __name__ == '__main__': main()
