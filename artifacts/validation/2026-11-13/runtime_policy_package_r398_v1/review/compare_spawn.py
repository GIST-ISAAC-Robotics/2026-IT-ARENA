"""실제 Lane/spawn: 원 체인 대 이관 체인의 계획24/current223 제한 비교."""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path
import time
import traceback

from review_support import (HERE, BASE, ROOT, corpus, selected_header, estimated_data,
                            encoded, digest, audit_factory)
from arena_autonomy.follow_trace import comparable_decision
from arena_autonomy.runtime_dual_process import Lane


def verify_manifest():
    manifest = json.loads((HERE / 'manifest.json').read_bytes())
    actual = {name: digest(ROOT / name) for name in manifest['files']}
    assert actual == manifest['files'], 'frozen review/source/input hash differs'
    return actual


def execute(stage, header, records, label, output_dir, cleanup):
    """한 번에 spawn 자식 하나. 이 비용은 정확비교용으로 성능 수치가 아니다."""
    lane = Lane(stage, header, capacity=4*1024*1024, timeout=20., evaluator_factory=audit_factory)
    results = []
    try:
        with (output_dir / (label + '.jsonl')).open('x', encoding='utf8') as stream:
            for index, row in enumerate(records):
                data = copy.deepcopy(row['data'])
                if header['runtime_time_policy'] == 'host_estimated_v1':
                    data = estimated_data(stage, data)
                before = encoded(data)
                lane.offer(data, row['generation'], row['sequence'], row['reset_reason'])
                deadline = time.monotonic() + 25.
                while True:
                    lane.connection.poll(.05)
                    result = lane.collect()
                    if result is not None:
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError(label + ' comparison wait exceeded')
                assert encoded(data) == before, 'parent snapshot changed'
                value = result['output']
                # 기존 공통 계약의 두 surface-fit processing_ms 위치만 정규화한다.
                comparable = dict(value)
                if stage == 'plan':
                    comparable['output'] = comparable_decision(value['output'])
                results.append(comparable)
                stream.write(json.dumps(dict(index=index, trace_seq=row['trace_seq'],
                    result=value, generation=result['generation'], sequence=result['sequence']),
                    allow_nan=False) + '\n')
                stream.flush()
    finally:
        cleanup.append(dict(label=label, **lane.close()))
    return results


def first_difference(a, b, path='$'):
    if type(a) is not type(b):
        return path + ': type differs'
    if isinstance(a, dict):
        if set(a) != set(b):
            return path + ': keys differ'
        for key in a:
            difference = first_difference(a[key], b[key], path + '.' + str(key))
            if difference:
                return difference
        return None
    if isinstance(a, list):
        if len(a) != len(b):
            return path + ': length differs'
        for index, (left, right) in enumerate(zip(a, b)):
            difference = first_difference(left, right, path + '[' + str(index) + ']')
            if difference:
                return difference
        return None
    return None if a == b else path + ': value differs'


def compare(label, left, right, details):
    assert len(left) == len(right)
    for index, (before, after) in enumerate(zip(left, right)):
        difference = first_difference(before, after)
        details.append(dict(comparison=label, index=index, equal=difference is None,
                            first_difference=difference))
        if difference:
            raise AssertionError(label + ' row ' + str(index) + ' ' + difference)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--numeric', choices=('python', 'native_v1'), default='native_v1')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    destination = args.output.resolve()
    assert destination.is_relative_to(BASE) and not destination.exists(), 'new artifact result directory required'
    source_before = verify_manifest()
    destination.mkdir(parents=True)
    cleanup, comparisons, error = [], [], None
    report = dict(passed=False, numeric=args.numeric, cleanup=cleanup, comparisons=comparisons,
        scope='saved-input same-common/final-geometry differential; not old-run replay, performance, ROS or hardware proof')
    try:
        recorded_header, plans, currents = corpus()
        header = selected_header(recorded_header, args.numeric)
        report['header'] = header
        report['plan_prefix_count'] = 24
        report['current_count'] = len(currents)
        report['recorded_current_modes'] = dict(Counter(
            row['data']['output'][2]['normal']['selector_mode'] for row in currents))
        report['recorded_current_expected_used_as_oracle'] = False
        # 모든 체인을 별도의 실제 spawn에서 시작하므로 전역 plan patch가 섞이지 않는다.
        old_plan = execute('plan', dict(header, review_chain='original'), plans[:24],
                           'original_plan24', destination, cleanup)
        new_plan = execute('plan', dict(header, review_chain='staged'), plans[:24],
                           'staged_plan24', destination, cleanup)
        compare('relocated_plan', old_plan, new_plan, comparisons)
        report['plan_prefix_checkpoints'] = {'12': True, '24': True}
        old_current = execute('current', dict(header, review_chain='original'), currents,
                              'original_current223', destination, cleanup)
        new_current = execute('current', dict(header, review_chain='staged'), currents,
                              'staged_current223', destination, cleanup)
        compare('relocated_current', old_current, new_current, comparisons)
        estimated_header = dict(header, review_chain='staged', runtime_time_policy='host_estimated_v1')
        estimated_plan = execute('plan', estimated_header, plans[:24],
                                 'estimated_plan24', destination, cleanup)
        compare('same_values_time_policy_plan', old_plan, estimated_plan, comparisons)
        selected = [0, 60, 119, 123, 125, 222]
        estimated_current = execute('current', estimated_header, [currents[i] for i in selected],
                                    'estimated_current6', destination, cleanup)
        compare('same_values_time_policy_current', [old_current[i] for i in selected],
                estimated_current, comparisons)
        report['estimated_current_indices'] = selected
        assert all(not item['forced'] and not item['alive'] and item['exitcode'] == 0 for item in cleanup)
        report['passed'] = True
    except BaseException:
        error = traceback.format_exc()
        report['error'] = error
    finally:
        report['source_and_inputs_unchanged'] = verify_manifest() == source_before
        report['passed'] = report['passed'] and report['source_and_inputs_unchanged']
        (destination / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    print(json.dumps({key: report.get(key) for key in (
        'passed', 'numeric', 'plan_prefix_count', 'current_count', 'source_and_inputs_unchanged', 'error')}, ensure_ascii=False))
    if error or not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
