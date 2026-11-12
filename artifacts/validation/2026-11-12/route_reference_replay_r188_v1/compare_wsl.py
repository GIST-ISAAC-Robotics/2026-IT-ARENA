"""r187 저장 입력에서 실제 후보 재현과 원 경로 정책 대조. 제어 연결 없음."""
import hashlib
import importlib.util
import json
from collections import Counter
from pathlib import Path

from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output
from arena_autonomy.local_follow import LocalFollow

A = Path(__file__).resolve().parent
RUN = A.parent / 'expiry_route_runtime_r187_v1/two_actor_run1/run1'
TRACE = RUN / 'follow_decisions.jsonl'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def brief(value):
    normal = value[2].get('normal', {})
    return dict(speed=value[0], steering=value[1], mode=normal.get('selector_mode'),
                reason=normal.get('reason'), selector_reason=normal.get('selector_reason'),
                path_wall=normal.get('path_wall'), selection=normal.get('boundary_selection'))


def main():
    load('r188_parent', A.parent / 'guard_approach_cap_r181_v1/candidate.py').install()
    candidate = load('r188_route', A.parent / 'expiry_route_reference_r186_v1/candidate_v2.py')
    original_method = LocalFollow._baseline_command
    events = [json.loads(line) for line in TRACE.open()]
    outputs = {}
    mismatches = []
    for mode in ('original_route', 'recorded_candidate'):
        LocalFollow._baseline_command = original_method
        if mode == 'recorded_candidate':
            candidate.install()
        owner, motion = make_controller(events[0]['data'])
        outputs[mode] = {}
        for event in events[1:]:
            data = event['data']
            if event['kind'] == 'reset':
                if mode == 'recorded_candidate' and data.get('reason') == 'decision_source_expired':
                    candidate.reset_reference(owner)
                else:
                    owner.reset()
            if event['kind'] != 'call':
                continue
            result = freeze(replay_call(owner, data['input'], motion))
            outputs[mode][event['seq']] = result
            if mode == 'recorded_candidate' and not equal_output(
                    comparable_decision(result), comparable_decision(data['output'])):
                mismatches.append(event['seq'])
    rows = []
    counts = Counter()
    for event in events:
        if event['kind'] != 'call':
            continue
        seq = event['seq']
        before, after = (outputs[name][seq] for name in ('original_route', 'recorded_candidate'))
        if not equal_output(comparable_decision(before), comparable_decision(after)):
            counts['full_output_changes'] += 1
            counts['new_stop'] += before[0] > 0 and after[0] <= 0
            counts['new_positive'] += before[0] <= 0 and after[0] > 0
            counts['command_changes'] += before[:2] != after[:2]
            rows.append(dict(seq=seq, t=event['data']['input']['reference_s'],
                             before=brief(before), after=brief(after)))
    trajectory = [json.loads(line) for line in (RUN / 'trajectory.jsonl').open()]
    low = []; active = []
    for item in trajectory:
        if item.get('ground_speed_mps', 0) < .1:
            active.append(item)
        elif active:
            low.append(active); active = []
    if active:
        low.append(active)
    windows = []
    for block in sorted(low, key=lambda v: v[-1]['t']-v[0]['t'], reverse=True)[:5]:
        start, end = block[0]['t'], block[-1]['t']
        reasons = Counter()
        for event in events:
            if event['kind'] == 'call' and start <= event['data']['input']['reference_s'] <= end:
                n = event['data']['output'][2].get('normal', {})
                reasons[str((n.get('selector_mode'), n.get('reason'), n.get('selector_reason')))] += 1
        windows.append(dict(start=start, end=end, sampled_span=end-start,
                            progress=block[0]['progress_m'], reasons=dict(reasons)))
    result = dict(calls=len(outputs['recorded_candidate']), mismatches=mismatches,
                  counts=dict(counts), changes=rows, low_windows=windows,
                  trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),
                  scope='recorded input only; low windows use sampled trajectory, not registered dense gate')
    with (A / 'result_wsl.json').open('x') as out:
        json.dump(result, out, indent=2); out.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('changes', 'low_windows')}))
    print(json.dumps(windows))
    assert not mismatches


if __name__ == '__main__':
    main()

