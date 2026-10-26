"""직선 중앙 복귀 보류의 기존 피추월/코너 저장 입력 대조. 새로운 주행 증거가 아니다."""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import time
import zipfile
from arena_autonomy.follow_trace import freeze, replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.m3_execution_trial import ExecutionConfig, M3ExecutionFollow
from compare_m3_straight_priority import compare
from probe_grid_follow import ROOT


def replay(events, enabled, priority):
    header = events[0]['data']
    config = replace(ExecutionConfig(**header['execution_config']),
                     straight_with_threat=priority, defer_center_on_straight=enabled)
    controller = M3ExecutionFollow(FollowConfig(**header['follow_config']), config,
                                   allow_simulation_trial=True)
    motion = MotionConfig(**header['motion_config'])
    rows = []
    for event in events[1:-1]:
        if event['kind'] == 'reset':
            controller.reset(); rows.append(dict(reset=True))
        elif event['kind'] == 'call':
            before = time.perf_counter_ns()
            output = freeze(replay_call(controller, event['data']['input'], motion))
            rows.append(dict(output=output, reference_s=event['data']['input']['reference_s'],
                             cost_ms=(time.perf_counter_ns()-before)/1e6))
        else:
            raise ValueError('unexpected stored event')
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    out = parser.parse_args().output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'): parser.error('new validation output required')
    out.mkdir(parents=True, exist_ok=False)
    base = ROOT/'artifacts/validation/2026-10-26/m5_start_constraints_v1'
    sources = {Path(__file__), ROOT/'scripts/compare_m3_straight_priority.py'}
    sources.update((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    frozen = {p.relative_to(ROOT).as_posix():p.read_bytes() for p in sources}
    with zipfile.ZipFile(out/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as z:
        for name, raw in frozen.items(): z.writestr(name, raw)
    prior_path = base/'run21_priority_saved/data/report.json'
    prior_raw = prior_path.read_bytes(); prior = json.loads(prior_raw)['cases']
    names = ['run17_observed_release_closed_loop', 'run21_right_encounter', 'run19_stop_restart',
             'run32_corner_after_threat', 'run35_committed_surface_corner']
    summaries, details = {}, {}
    for name in names:
        path = ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1'/name/'run/follow_decisions.jsonl'
        raw = path.read_bytes(); events = [json.loads(line) for line in raw.decode().splitlines()]
        priority = name in prior
        before = replay(events, False, priority); after = replay(events, True, priority)
        diffs = []
        expected_path = (ROOT/'artifacts/validation/2026-10-26/m3_rear_residual_v1/cachefix_runs/'
                         'run7_premises_cachefix'/(name+'__plain.json'))
        expected_raw = None if priority else expected_path.read_bytes()
        expected = prior[name]['on'] if priority else json.loads(expected_raw)
        if len(expected) != len(before): raise ValueError('expected event alignment lost')
        for i, (row, ref) in enumerate(zip(before, expected)):
            if row.get('reset'): same = ref.get('reset') is True
            elif priority: same = row['output'] == ref['output']
            else: same = hashlib.sha256(json.dumps(row['output'], sort_keys=True).encode()).hexdigest() == ref['sha']
            if not same: diffs.append(i)
        result = compare(before, after)
        result.update(off_expected_differences=diffs, input_unchanged=path.read_bytes()==raw,
                      input_sha256=hashlib.sha256(raw).hexdigest(), straight_priority=priority,
                      expected_unchanged=prior_path.read_bytes()==prior_raw if priority else expected_path.read_bytes()==expected_raw)
        selected = [r['output'][2] for r in after if not r.get('reset') and
                    r['output'][2].get('m3_lateral',{}).get('deferred_center',{}).get('ok')]
        result['defer_eligible_calls'] = len(selected)
        result['defer_large_offset_calls'] = sum(abs(r['m3_lateral']['state'].get('d_rear0',0))>=.08 for r in selected)
        details[name] = dict(off=before, on=after, comparison=result)
        summaries[name] = dict(result, changed_actions_count=len(result['changed_actions']),
                               changed_actions=result['changed_actions'][:8])
        print(json.dumps({name:summaries[name]}), flush=True)
    unchanged = all((ROOT/n).read_bytes()==b for n,b in frozen.items())
    passed = unchanged and all(c['input_unchanged'] and c['expected_unchanged'] and
                               not c['off_expected_differences'] for c in summaries.values())
    (out/'report.json').write_text(json.dumps(details,ensure_ascii=False), encoding='utf-8')
    (out/'summary.json').write_text(json.dumps(dict(cases=summaries, sources_unchanged=unchanged,
        integrity_pass=passed, source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
        scope='fixed sensor inputs; changed actions do not update the following scene'),ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if passed else 1


if __name__ == '__main__': raise SystemExit(main())
