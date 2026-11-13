"""한 번의 r375 비용 진단. ARM 내부 불변과 x86 재현은 별개이며 원 실패를 대체하지 않는다."""
import argparse
import collections
import copy
import importlib.util
import json
from pathlib import Path
import platform
import sys
import time

from support import A, ROOT, EXPECTED, digest, load_comparison, verify_manifest, check_imports
from diff_utils import comparison, recorded_result


def load_r380():
    path = A.parent/'current_cost_r380_v1/run.py'
    spec = importlib.util.spec_from_file_location('r382_frozen_r380_measurement', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def same_arm(compare, record, original, before, after):
    # r375 compare_record의 같은 아키텍처 불변 조건 그대로. x86 비교만 아래로 분리한다.
    assert compare.equal_output(record, original), 'input changed'
    retry = after.get(compare.policy.DIAGNOSTIC) or {}
    applied = retry.get('applied') is True
    if applied:
        assert record['output'][2]['normal']['selector_mode'] == 'RELEASED'
        assert record['output'][2]['normal']['selector_choice'] == 'parallel'
        assert before['speed_mps'] == 0 and 0 < after['speed_mps'] <= record['output'][0] <= .10
        assert all(before.get(k) == after.get(k) for k in (
            'steering_rad', 'reference_s', 'path_source_s', 'original_reference_s', 'path_points'))
        assert before['current_route_proof']['region'] == after['current_route_proof']['region']
        assert after['current_route_proof']['accepted'] and not retry['unknown_filled']
        assert after['current_side_presence'] == {'left': False, 'right': False}
        assert after['maximum_body_corner_lateral_m'] <= .01
    else:
        assert compare.equal_output(before, {k:v for k,v in after.items() if k != compare.policy.DIAGNOSTIC}), 'same ARM non-applied output changed'
    if record['label'] != 'r371':
        assert not applied, 'non-target candidate applied'
    return dict(index=None, label=record['label'], reference_s=record['current']['reference_s'],
        mode=record['output'][2]['normal'].get('selector_mode'),
        choice=record['output'][2]['normal'].get('selector_choice'), applied=applied,
        original_target_mps=record['output'][0], measured_speed_mps=record['current']['speed_mps'],
        before_speed_mps=before['speed_mps'], after_speed_mps=after['speed_mps'],
        before_reason=before['reason'], after_reason=after['reason'], diagnostic=retry)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-machine', choices=('aarch64', 'any'), default='aarch64')
    args = parser.parse_args()
    assert args.require_machine == 'any' or platform.machine() == args.require_machine
    output = args.output.resolve()
    assert output.is_relative_to(A) and not output.exists(), 'fresh r382 output required'
    output.mkdir()
    manifest = verify_manifest()
    compare = load_comparison()
    imports = check_imports(compare, 'native_v1')
    historical = load_r380()
    expected = json.loads(EXPECTED.read_bytes())
    records = historical.records_in_original_order(compare)
    assert len(records) == 572 and expected['passed'] and expected['original_current_reproduced'] == 217
    raw, checked, checks, negatives = [], [], [], []
    context = dict(phase='stored', index=0)
    original_evaluate = compare.evaluate
    error = None
    began_run = time.perf_counter_ns()

    def log_check(kind, index, label, reference, actual):
        delta = comparison(reference, actual)
        # 동결 comparator와 일치하는지도 확인해 진단 코드 자체의 판정 누락을 막는다.
        assert delta['existing_comparator_equal'] == compare.equal_output(reference, actual)
        checks.append(dict(kind=kind, index=index, label=label, **delta))

    def evaluate(record):
        original = copy.deepcopy(record)
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
        row = dict(phase=context['phase'], index=context['index'], label=record['label'],
            reference_s=current['reference_s'], path_source_s=normal.get('path_source_s'),
            selector_mode=normal.get('selector_mode'), selector_choice=normal.get('selector_choice'),
            applied=diagnostic.get('applied') is True, rebase_wall_ms=elapsed,
            before=before, after=after)
        raw.append(row)
        assert compare.equal_output(record, original), 'input changed'
        if not row['applied']:
            row['same_arm_nonapplied_diff'] = comparison(before, {k:v for k,v in after.items() if k != compare.policy.DIAGNOSTIC})
        if context['phase'] == 'counterexample':
            row['index'] = len([r for r in raw if r['phase'] == 'counterexample'])-1
            row['counterexample'] = expected['counterexamples'][row['index']]['name']
        return before, after

    compare.evaluate = evaluate
    try:
        for index, record in enumerate(records):
            context['index'] = index
            original = copy.deepcopy(record)
            before, after = evaluate(record)
            row = same_arm(compare, record, original, before, after)
            row['index'] = index
            if record['label'] == 'r371':
                restored, cap_precondition = recorded_result(before, record['expected'])
                if cap_precondition is not None:
                    checks.append(dict(kind='x86_parent_cap_precondition', index=index, label=record['label'], **cap_precondition))
                log_check('x86_original_current', index, record['label'], record['expected'], restored)
            log_check('x86_r375_comparison_row', index, record['label'], expected['rows'][index], row)
            checked.append(row)
        context.update(phase='counterexample', index=0)
        # 기존 x86 마지막 승인 결과의 gap 인덱스로 동일한 17변형을 고정한다.
        # 이 fixture는 반례 생성에만 쓰며 제어기 입력/진행 허가로 사용하지 않는다.
        fixture = {compare.policy.DIAGNOSTIC: expected['last']['diagnostic']}
        negatives = compare.counterexamples(records[216], fixture)
        assert len(negatives) == 17 and len(raw) == 589
        assert all(n['stopped'] for n in negatives), 'one of 17 counterexamples moved'
        for index, (row, negative, reference) in enumerate(zip(raw[572:], negatives, expected['counterexamples'])):
            row['counterexample'] = negative['name']
            log_check('x86_counterexample', index, negative['name'], reference, negative)
        verify_manifest()
    except Exception as exc:
        error = repr(exc)
        raise
    finally:
        compare.evaluate = original_evaluate
        for name, values in (('timing_rows.jsonl', raw), ('comparison_rows.jsonl', checks)):
            with (output/name).open('x', encoding='utf8') as stream:
                for row in values: stream.write(json.dumps(compare.freeze(row), ensure_ascii=False)+'\n')
        stored = [r for r in raw if r['phase'] == 'stored']
        primary = [r for r in stored if r['label'] == 'r371']
        groups = dict(stored_all=historical.summary(stored), r371_all=historical.summary(primary),
            r371_applied=historical.summary([r for r in primary if r['applied']]),
            r371_unapplied=historical.summary([r for r in primary if not r['applied']]),
            holdout_all=historical.summary([r for r in stored if r['label'] != 'r371']),
            counterexamples_all=historical.summary([r for r in raw if r['phase'] == 'counterexample']))
        for label in sorted({r['label'] for r in stored}):
            for mode in sorted({str(r['selector_mode']) for r in stored if r['label'] == label}):
                groups[label+':'+mode] = historical.summary([r for r in stored if r['label'] == label and str(r['selector_mode']) == mode])
        expected_mismatches = [r for r in checks if not r['existing_comparator_equal']]
        exact_mismatches = [r for r in checks if not r['exact_tree_equal']]
        exact_categories = collections.Counter(d['category'] for r in checks for d in r['differences'])
        beyond_categories = collections.Counter(d['category'] for r in checks for d in r['differences'] if not d['existing_comparator_equal'])
        sensitive = [dict(kind=r['kind'], index=r['index'], label=r['label'], **d)
            for r in checks for d in r['differences'] if d['category'] in (
                'command_or_measured_state','reason_or_selector','permission_or_boolean','time_or_source')]
        complete = error is None and len(checked) == 572 and len(negatives) == 17
        applied_count = sum(r['applied'] for r in primary)
        changed_sources = [n for n,h in manifest['files'].items() if not (ROOT/n).is_file() or digest(ROOT/n) != h]
        report = dict(schema='r382_diagnostic_v1', diagnostic_completed=complete,
            same_arm_invariants_passed=complete and not changed_sources,
            x86_existing_comparator_reproduction_passed=complete and not expected_mismatches,
            x86_exact_tree_reproduction_passed=complete and not exact_mismatches,
            automatic_adoption_allowed=False, original_r380_failure_preserved=True,
            error=error, completed_stored=len(checked), completed_counterexamples=len(negatives),
            applied_count=applied_count, x86_expected_applied_count=93,
            applied_count_matches_x86=applied_count == 93 and len(primary) == 217,
            x86_mismatches_by_existing_comparator=[{k:r[k] for k in ('kind','index','label','difference_count','categories')} for r in expected_mismatches],
            exact_difference_categories=dict(exact_categories), beyond_existing_comparator_categories=dict(beyond_categories),
            command_state_source_permission_exact_differences=sensitive,
            command_state_source_permission_beyond_existing_comparator=[d for d in sensitive if not d['existing_comparator_equal']],
            full_differences='comparison_rows.jsonl: every exact difference retained; no field filtering',
            x86_review_required=bool(expected_mismatches) or applied_count != 93,
            groups=groups, counterexamples=negatives, imports=imports, changed_sources=changed_sources,
            source_input_files=len(manifest['files']), input_manifest_sha256=digest(A/'source_manifest.json'),
            reference_comparison_sha256=digest(EXPECTED), total_wall_s=(time.perf_counter_ns()-began_run)/1e9,
            machine=platform.machine(), python=sys.version, clock='time.perf_counter_ns',
            measurement='top-level r375 only; internal retries included; same frozen r380 serial ordering',
            warmup='none discarded; external r370 immediately precedes r375; cache effect remains',
            counterexample_fixture='frozen x86 comparison last diagnostic; identical gap-index mutations',
            command_expiry_evaluated=False, deadline_permission=False, publication_evaluated=False,
            devices_opened=False, source_timestamps_renewed=False, default_changed=False,
            threshold_scope='100/200ms 계산 초과 수. 명령 만료/실차 안전 통과 아님.')
        (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
        print(json.dumps({k:report[k] for k in ('diagnostic_completed','same_arm_invariants_passed','x86_existing_comparator_reproduction_passed','applied_count','groups','error')}, ensure_ascii=False))
        if changed_sources and error is None: raise AssertionError(('source mutated', changed_sources))


if __name__ == '__main__': main()
