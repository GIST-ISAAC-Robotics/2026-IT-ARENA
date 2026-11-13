"""r371 현재 217개 재현과 선택 후보/비대상 저장 입력 대조. 발행·주행 없음."""
import argparse
import collections
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
spec = importlib.util.spec_from_file_location('r375_compare_policy', A/'candidate.py')
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)

from arena_autonomy.follow_trace import restore_call, freeze, equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.numeric_kernels import configure
from arena_autonomy.runtime_dual_process import plan_reference
from arena_autonomy.scene_motion import motion_scene

TRACE = A.parent/'relative_body_cad_drive_r371_v1/run1/follow_decisions.jsonl'
TRACE_HASH = '1f2339aac85303432e07deee6932aa3340b4ef65b8eb963b2be2f4cc70bec910'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_trace(path, label):
    records = []
    header = None
    for line in path.open(encoding='utf8'):
        event = json.loads(line)
        if event['kind'] == 'header': header = event['data']
        if event['kind'] != 'call' or 'current_rebase' not in event['data']: continue
        c = event['data']['current_rebase']; plan = c['compute']['plan']
        records.append(dict(label=label, header=header, output=plan['output'],
            submitted=plan_reference(plan['input']), current=c['input'], expected=c['output']))
    return records


def evaluate(record):
    cfg = FollowConfig(**record['header']['follow_config'])
    motion = MotionConfig(**record['header']['motion_config'])
    current = record['current']
    info, scan, history = restore_call(current, motion)
    scene = motion_scene(scan, history, info['reference_s'], timing_verified=info['timing_verified'],
                         epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'])
    args = (record['output'], record['submitted'], current, scene, history, cfg)
    before = freeze(policy.prior.rebase(*args, require_current_region=True))
    after = freeze(policy.rebase(*args, require_current_region=True))
    return before, after


def recorded_result(before, expected):
    # ROS 부모가 추가한 감속 상한만 기록대로 복원한다. 후보로 원 실패를 덮어쓰지 않는다.
    cap = expected.get('new_plan_deceleration_cap')
    if cap is None: return before
    assert equal_output(before['speed_mps'], cap['checked_target_mps'])
    return {**before, 'speed_mps': min(before['speed_mps'], cap['speed_mps']),
            'new_plan_deceleration_cap': cap}


def compare_record(index, record):
    original = copy.deepcopy(record)
    before, after = evaluate(record)
    assert equal_output(record, original), 'input changed'
    if record['label'] == 'r371':
        assert equal_output(recorded_result(before, record['expected']), record['expected']), (
            index, 'original current output mismatch')
    retry = after.get(policy.DIAGNOSTIC) or {}
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
        assert equal_output(before, {k: v for k, v in after.items() if k != policy.DIAGNOSTIC})
    return dict(index=index, label=record['label'], reference_s=record['current']['reference_s'],
        mode=record['output'][2]['normal'].get('selector_mode'),
        choice=record['output'][2]['normal'].get('selector_choice'), applied=applied,
        original_target_mps=record['output'][0], measured_speed_mps=record['current']['speed_mps'],
        before_speed_mps=before['speed_mps'], after_speed_mps=after['speed_mps'],
        before_reason=before['reason'], after_reason=after['reason'], diagnostic=retry), after


def counterexamples(original, accepted):
    """마지막 저장 입력을 한 가지씩 바꾼 계산 반례. 물리 장면 생성은 아니다."""
    cases = []
    def add(name, mutate, must_stop=False):
        row = copy.deepcopy(original)
        mutate(row)
        before, after = evaluate(row)
        assert not (after.get(policy.DIAGNOSTIC) or {}).get('applied'), name
        assert equal_output(before, {k: v for k, v in after.items() if k != policy.DIAGNOSTIC}), name
        if must_stop: assert after['speed_mps'] == 0, (name, after['reason'])
        cases.append(dict(name=name, stopped=after['speed_mps'] == 0,
            baseline_equal=True, reason=after['reason'], speed_mps=after['speed_mps']))
    add('measured_speed_above_scope', lambda r: r['current'].__setitem__('speed_mps', .151))
    add('reverse_measured_speed', lambda r: r['current'].__setitem__('speed_mps', -.001))
    add('target_above_scope', lambda r: r['output'].__setitem__(0, .101))
    add('dt_outside_scope', lambda r: r['current'].__setitem__('dt_s', .101), True)
    add('steering_outside_cad', lambda r: r['current'].__setitem__('previous_steering_rad', .46))
    add('path_source_expired', lambda r: r['output'][2]['normal'].__setitem__(
        'path_source_s', r['current']['reference_s']-.501), True)
    add('submitted_epoch_changed', lambda r: r['submitted'].__setitem__('epoch', 999), True)
    add('selected_path_changed', lambda r: r['output'][2]['normal']['path_points'][0].__setitem__(0, 99.), True)
    add('parallel_side_departure', lambda r: r['output'][2]['normal']['parallel_hold_trial'].__setitem__(
        'side_departure', {'applied': True}), True)
    add('boundary_from_memory', lambda r: r['output'][2]['normal'].__setitem__('boundary_from_memory', True))
    add('body_gap_option_disabled', lambda r: r['header']['follow_config'].__setitem__('released_body_gap_trial', False))
    gap = accepted[policy.DIAGNOSTIC]['proof']['gaps'][0]
    def raw_gap(r, value):
        raw = r['current']['scan']['measurements']['ranges_m']
        for i in range(gap['first'], gap['last']+1): raw[i] = value
    add('nan_instead_of_positive_infinity', lambda r: raw_gap(r, {'nonfinite': 'nan'}), True)
    add('too_near_finite_gap', lambda r: raw_gap(r, .02), True)
    add('near_outer_neighbor', lambda r: r['current']['scan']['measurements']['ranges_m'].__setitem__(
        gap['first']-1, .08), True)
    def front(r):
        raw = r['current']['scan']['measurements']
        for i, angle in enumerate(raw['angles_rad']):
            if abs(math.remainder(angle, math.tau)) < math.radians(20): raw['ranges_m'][i] = .16
    add('new_front_obstacle', front, True)
    for sign, name in ((1, 'left'), (-1, 'right')):
        def side(r, sign=sign):
            raw = r['current']['scan']['measurements']
            for i, angle in enumerate(raw['angles_rad']):
                if abs(math.remainder(angle-sign*math.pi/2, math.tau)) < math.radians(25):
                    raw['ranges_m'][i] = .18
        add('new_'+name+'_side_returns', side, True)
    return cases


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=('python', 'native_v1'), default='native_v1')
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    destination = args.output_dir or A/('results_'+args.backend)
    destination.mkdir(exist_ok=False)
    configure(args.backend)
    inputs = [TRACE, A.parent/'released_crawl_drive_r364_v1/run1/follow_decisions.jsonl',
        A.parent/'current_input_jetson_r296_v1/headers.json',
        A.parent/'current_input_jetson_r296_v1/selected_inputs.json']
    hashes = {str(p.relative_to(ROOT)): digest(p) for p in inputs}
    assert digest(TRACE) == TRACE_HASH
    records = load_trace(TRACE, 'r371')
    assert len(records) == 217
    primary_count = len(records)
    records.extend(load_trace(inputs[1], 'r364_non_target'))
    headers = json.loads(inputs[2].read_bytes())
    for selected in json.loads(inputs[3].read_bytes()):
        row = selected['input']; label = selected['source']
        records.append(dict(label='holdout_'+label, header=headers[label], output=row['output'],
            submitted=plan_reference(row['input']), current=row['current_rebase']['input']))
    rows = []; final = None
    try:
        for index, record in enumerate(records):
            row, after = compare_record(index, record)
            if record['label'] != 'r371': assert not row['applied'], (index, 'holdout changed')
            rows.append(row)
            if index == primary_count-1: final = after
        assert final is not None and (final.get(policy.DIAGNOSTIC) or {}).get('applied'), 'last failure not resolved'
        negatives = counterexamples(records[primary_count-1], final)
        assert hashes == {str(p.relative_to(ROOT)): digest(p) for p in inputs}, 'input hash changed'
        groups = {label: dict(count=sum(r['label'] == label for r in rows),
            applied=sum(r['applied'] for r in rows if r['label'] == label)) for label in sorted({r['label'] for r in rows})}
        source_hashes = {}
        for module in list(sys.modules.values()):
            filename = getattr(module, '__file__', None)
            if not filename: continue
            path = Path(filename).resolve()
            if path.is_file() and path.is_relative_to(ROOT) and path.suffix in ('.py', '.so', '.pyd'):
                source_hashes[path.relative_to(ROOT).as_posix()] = digest(path)
        with zipfile.ZipFile(destination/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
            for name in source_hashes: archive.write(ROOT/name, name)
        result = dict(passed=True, backend=args.backend, groups=groups,
            original_current_reproduced=primary_count, input_hashes=hashes,
            input_hashes_unchanged=True, source_hashes=source_hashes,
            primary_modes=dict(collections.Counter(r['mode'] for r in rows[:primary_count])),
            last=rows[primary_count-1], counterexamples=negatives, rows=rows,
            scope='저장 입력/계산 반례만. 새 주행, 실차 안전, 기본 채택, Jetson 처리율 증명이 아님.')
        (destination/'comparison.json').write_text(json.dumps(freeze(result), ensure_ascii=False, indent=2)+'\n', encoding='utf8')
        print(json.dumps(dict(passed=True, groups=groups, counterexamples=len(negatives), last=rows[primary_count-1]), ensure_ascii=False))
    except Exception as error:
        (destination/'failure.json').write_text(json.dumps(freeze(dict(passed=False,
            error=repr(error), completed_rows=rows)), ensure_ascii=False, indent=2)+'\n', encoding='utf8')
        raise


if __name__ == '__main__': main()
