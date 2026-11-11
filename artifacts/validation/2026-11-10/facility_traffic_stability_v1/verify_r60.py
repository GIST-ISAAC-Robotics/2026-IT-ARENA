"""연결 코드와 저장 후보의 명령 일치, 직전 시설 교통 기록의 변경 범위를 확인한다."""
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PREVIOUS = ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'
sys.path.insert(0, str(PREVIOUS))
from replay_tools import build_controller, read_events
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision


def verify(path, dest, candidate=None):
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    original = digest(path)
    expected = ([json.loads(line) for line in candidate.open()] if candidate else None)
    stats, reasons, examples = Counter(), Counter(), []
    with dest.open('x', encoding='utf8') as stream:
        for event in read_events(path):
            data = event['data']
            if event['kind'] == 'header': controller, motion = build_controller(data)
            elif event['kind'] == 'reset': controller.reset()
            elif event['kind'] == 'call':
                out = freeze(replay_call(controller, data['input'], motion))
                old = data['output']; detail = out[2].get('normal') or out[2]
                probe = detail.get('selector_guard_candidate') or {}
                changed = not equal_output(out[:2], old[:2])
                if expected:
                    want = expected[stats['calls']]['current_path']
                    stats['candidate_command_mismatch'] += not equal_output(out[:2], want['command'])
                stats['calls'] += 1
                stats['recorded_command_changes'] += changed
                stats['recorded_full_changes'] += not equal_output(comparable_decision(out), comparable_decision(old))
                stats['recorded_zeros'] += old[0] <= 1e-6
                stats['new_zeros'] += out[0] <= 1e-6
                if changed:
                    row = dict(t=data['input']['reference_s'], before=old[:2], after=out[:2],
                        old_reason=(old[2].get('normal') or old[2]).get('reason'),
                        reason=detail.get('reason'), selector=detail.get('selector_reason'),
                        choice=detail.get('selector_choice'), trigger=probe.get('trigger'), accepted=probe.get('accepted'))
                    reasons[(row['old_reason'], row['reason'], row['selector'])] += 1
                    stream.write(json.dumps(row)+'\n')
                    if len(examples) < 8: examples.append(row)
    if expected: assert stats['calls'] == len(expected)
    return dict(input=str(path), stats=dict(stats), examples=examples,
                change_reasons=[dict(old=k[0], new=k[1], selector=k[2], count=v) for k,v in reasons.items()],
                input_sha256=original, input_unchanged=digest(path)==original)


if __name__ == '__main__':
    folder = HERE/'r60_connected'; folder.mkdir(exist_ok=False)
    results = {}
    results['c5'] = verify(PREVIOUS/'c_stationary_posts/run5/follow_decisions.jsonl', folder/'c5_changes.jsonl',
                           HERE/'c5_guard_probe/rows.jsonl')
    results['facility_traffic'] = verify(HERE/'two_actor_run1/run1/follow_decisions.jsonl', folder/'traffic_changes.jsonl')
    (folder/'summary.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps({k:dict(stats=v['stats'], input_unchanged=v['input_unchanged']) for k,v in results.items()}), flush=True)
