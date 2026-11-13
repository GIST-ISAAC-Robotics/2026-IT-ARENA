"""동결 r386 final 223개와 지정 반례 3개만 대조. ROS/장치/주행 없음."""
import argparse
import copy
from dataclasses import asdict, replace
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

import yaml

HERE = Path(__file__).resolve().parent
SOURCE = Path(os.environ['R386_SOURCE_ROOT']).resolve()
INPUT = Path(os.environ['R386_INPUT_ROOT']).resolve()
R386 = SOURCE.parent/'r386'
sys.path.insert(0, str(R386))
from support import load, original_helpers, verify

COUNTEREXAMPLES = ('current_time_unverified', 'expired_path_source',
                   'multiple_front_and_side_obstacles')


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def inputs(compare):
    """원 compare_three와 같은 순서/입력 복원; 795개 집합은 읽거나 실행하지 않는다."""
    path = INPUT/'artifacts/validation/2026-11-13/released_cad_drive_r377_v1/run1/follow_decisions.jsonl'
    result = []; header = None
    with path.open(encoding='utf8') as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'header': header = event['data']
            if event['kind'] != 'call' or 'current_rebase' not in event['data']: continue
            assert header is not None
            current = event['data']['current_rebase']; plan = current['compute']['plan']
            result.append(dict(index=len(result), event_seq=event['seq'], label='r377', header=header,
                output=plan['output'], submitted=compare.plan_reference(plan['input']), current=current['input']))
    assert len(result) == 223
    assert [result[i]['event_seq'] for i in (123, 125, 222)] == [606, 610, 804]
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    destination = parser.parse_args().output.resolve()
    assert destination.is_relative_to(HERE) and not destination.exists()
    destination.mkdir(); verify(); compare, imports = load('native_v1')
    from arena_autonomy.lidar_motion import MotionUnavailable
    from arena_autonomy.local_scene import blind_spec
    from arena_autonomy.vehicle_geometry import planning_parameters, preview_parameters
    from range_mask_component import bounded_adjacent_structure as component
    budget_candidate = module('r396_original_budget_candidate', R386/'candidate.py')
    assert Path(component.__code__.co_filename).resolve() == HERE/'staged/range_mask_component.py'
    selected = yaml.safe_load((SOURCE/'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml').read_bytes())['vehicle']
    planning, sender = planning_parameters(selected), preview_parameters(selected)
    expected = [json.loads(line) for line in (HERE/'expected_final_223.jsonl').read_text(encoding='utf8').splitlines()]
    assert [r['index'] for r in expected] == list(range(223))
    expected_three = json.loads((HERE/'expected_three_v2.json').read_bytes())
    helpers = original_helpers(); records = inputs(compare)
    rows = []; negatives = []; error = None; source_preserved = False

    def evaluate(record, phase, name, stream):
        original = copy.deepcopy(record); index = record['index']; current = record['current']
        cfg = compare.FollowConfig(**record['header']['follow_config'])
        assert (cfg.length_m, cfg.width_m, cfg.wheelbase_m, cfg.max_steering_rad) == (
            planning['vehicle_length_m'], planning['vehicle_width_m'], sender['wheelbase_m'], sender['max_steering_angle_rad'])
        cfg = replace(cfg, track_m=planning['vehicle_track_width_m'],
            wheel_radius_m=planning['vehicle_wheel_radius_m'], wheel_width_m=planning['vehicle_wheel_width_m'])
        info, raw, history = compare.restore_call(current, compare.MotionConfig(**record['header']['motion_config']))
        try:
            scene = compare.motion_scene(raw, history, info['reference_s'], timing_verified=info['timing_verified'],
                epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'])
        except MotionUnavailable as exc:
            # 원 r386 실패와 동일한 입력 생성 거절만 분리한다. 다른 예외는 실패다.
            if phase != 'counterexample' or name != 'current_time_unverified' or exc.args != ('scan_timing_unverified',):
                raise
            assert record == original
            row = dict(phase=phase, name=name, index=index, input_rejected=True,
                stage='motion_scene', exception=repr(exc), baseline_evaluated=False,
                candidate_evaluated=False, stop_command_evaluated=False, passed=True)
            stream.write(json.dumps(row, ensure_ascii=False)+'\n'); stream.flush()
            return row
        assert name != 'current_time_unverified', 'unverified scene unexpectedly constructed'
        profile = info['rear_half_angle_deg']; sectors = blind_spec(profile)[1]
        call = (record['output'], record['submitted'], current, scene, history, cfg)
        start = time.perf_counter_ns()
        baseline = compare.policy.rebase(*call, require_current_region=True)
        baseline_ms = (time.perf_counter_ns()-start)/1e6
        baseline_frozen = compare.freeze(baseline)
        if phase == 'stored':
            assert baseline_frozen == expected[index]['before'], ('frozen final baseline mismatch', index)
            assert asdict(cfg) == expected[index]['actual_config'], ('final config mismatch', index)
            assert current['reference_s'] == expected[index]['reference_s']
        eligible = budget_candidate.eligible(baseline, record['output'], current, cfg, True)
        proof_calls = 0

        def prove(s, scan, q, pad, length, width, speed, target, **unused):
            nonlocal proof_calls
            proof_calls += 1
            return component(s, scan, q, pad, length, width, speed, target, sectors,
                mask_profile=profile, motion_epoch=info['epoch'], timing_verified=info['timing_verified'])

        old_proof = budget_candidate.bounded_body_nonreturn
        costs = {}; start = time.perf_counter_ns()
        try:
            budget_candidate.bounded_body_nonreturn = prove
            combined = budget_candidate.try_from_current_result(baseline, *call,
                core_rebase=compare.policy.core_policy.rebase, require_current_region=True, costs=costs)
        finally:
            budget_candidate.bounded_body_nonreturn = old_proof
        added_ms = (time.perf_counter_ns()-start)/1e6
        assert compare.freeze(baseline) == baseline_frozen, 'baseline input mutated'
        assert record == original, 'saved input mutated'
        after = compare.freeze(combined); diagnostic = after.get(budget_candidate.DIAGNOSTIC) or {}
        applied = diagnostic.get('applied') is True
        assert proof_calls <= 1
        assert diagnostic.get('additional_target_attempts', 0) <= 1
        assert diagnostic.get('final_full_checks', 0) <= 1
        if applied:
            assert eligible and baseline['speed_mps'] == 0 < combined['speed_mps'] <= .10
            assert diagnostic['additional_target_attempts'] == diagnostic['final_full_checks'] == 1
            assert all(baseline.get(k) == combined.get(k) for k in (
                'steering_rad', 'reference_s', 'path_source_s', 'original_reference_s', 'path_points'))
            assert combined['current_route_proof']['accepted']
            assert diagnostic['region']['distance_m'] <= diagnostic['proposal']['allowed_distance_m']
            assert not any(diagnostic[k] for k in ('unknown_filled', 'source_timestamp_renewed', 'state_transition', 'default_changed'))
            if diagnostic['region']['unobserved']:
                assert diagnostic['proof']['accepted'] and not diagnostic['proof']['unknown_filled']
        else:
            assert {k:v for k,v in after.items() if k != budget_candidate.DIAGNOSTIC} == baseline_frozen, 'nonapplied output changed'
        if not eligible: assert after == baseline_frozen, 'non-target output changed'
        three_exact = None
        if phase == 'stored' and str(index) in expected_three:
            assert after == expected_three[str(index)], ('prior three-input candidate mismatch', index)
            three_exact = True
        negative_passed = None
        if phase == 'counterexample':
            negative_passed = not applied and combined['speed_mps'] == 0
            assert negative_passed, ('counterexample candidate accepted or moving', name)
        row = dict(phase=phase, name=name, index=index, event_seq=record['event_seq'], reference_s=current['reference_s'],
            mode=record['output'][2]['normal'].get('selector_mode'), eligible=eligible, applied=applied,
            baseline_exact=True if phase == 'stored' else None, prior_three_exact=three_exact,
            input_unchanged=True, baseline_evaluated=True, candidate_evaluated=True,
            input_rejected=False, passed=negative_passed, proof_calls=proof_calls,
            baseline=baseline_frozen, candidate=after, baseline_wall_ms=baseline_ms,
            added_wall_ms=added_ms, comparison_total_wall_ms=baseline_ms+added_ms, **costs)
        stream.write(json.dumps(row, ensure_ascii=False)+'\n'); stream.flush()
        return row

    try:
        with (destination/'rows.jsonl').open('x', encoding='utf8') as stream:
            for record in records: rows.append(evaluate(record, 'stored', None, stream))
            # 원 생성기를 수정 없이 읽되 새로 회복한 마지막 입력의 세 반례만 평가한다.
            cases = dict(helpers.counterexample_records(records[-1]))
            for name in COUNTEREXAMPLES:
                negatives.append(evaluate(cases[name], 'counterexample', name, stream))
        verify(); source_preserved = True
    except Exception as exc:
        error = repr(exc)
    finally:
        try: verify(); source_preserved = True
        except Exception as exc:
            source_preserved = False
            error = (error+'; ' if error else '')+repr(exc)
        recovered = [r['index'] for r in rows if r['baseline']['speed_mps'] == 0 < r['candidate']['speed_mps']]
        stages = ('baseline_wall_ms', 'added_wall_ms', 'comparison_total_wall_ms',
                  'proposal_geometry_ms', 'proposal_classification_ms', 'final_core_ms', 'final_region_ms', 'final_gap_ms')
        groups = {name:{stage:helpers.summary(group, stage) for stage in stages} for name,group in (
            ('all_current', rows), ('eligible', [r for r in rows if r['eligible']]),
            ('applied', [r for r in rows if r['applied']]), ('non_target', [r for r in rows if not r['eligible']]))}
        report = dict(passed=error is None and source_preserved and len(rows)==223 and len(negatives)==3,
            error=error, completed_current=len(rows), completed_counterexamples=len(negatives),
            baseline_exact_count=sum(r['baseline_exact'] is True for r in rows),
            prior_three_exact_indices=[r['index'] for r in rows if r['prior_three_exact']],
            baseline_positive=sum(r['baseline']['speed_mps']>0 for r in rows),
            candidate_positive=sum(r['candidate']['speed_mps']>0 for r in rows),
            recovery_indices=recovered, eligible_count=sum(r['eligible'] for r in rows),
            applied_count=sum(r['applied'] for r in rows),
            non_target_unchanged_count=sum(not r['eligible'] for r in rows),
            nonapplied_unchanged_count=sum(not r['applied'] for r in rows),
            counterexamples=[{k:v for k,v in r.items() if k not in ('baseline','candidate')} for r in negatives],
            groups=groups, imports=imports, source_preserved=source_preserved,
            timing_scope='single x86 serial replay; added candidate work only; no deadline or live-loop claim',
            original_r386_pipeline_passed=False, legacy795_replayed=False, original795_replayed=False,
            unit29_repeated=False, helper_only_arm_repeated=False,
            new_default=False, drive_executed=False, runtime_authorized=False, r386_adopted=False)
        (destination/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
        print(json.dumps({k:v for k,v in report.items() if k not in ('groups','counterexamples')}, ensure_ascii=False))
    if not report['passed']: raise SystemExit(1)


if __name__ == '__main__': main()
