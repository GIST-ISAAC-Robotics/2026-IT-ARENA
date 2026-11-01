"""운동 입력 대입 재생의 전 구간 민감도를 집계한다(개방 루프, 진단 전용)."""
import collections
import gzip
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
PAIRS = {'rise_only': ('rise_only_baseline', 'rise_only_truth_history', None),
         'turn_only': ('turn_only_baseline', 'turn_only_truth_history', None),
         'combined_failed': ('combinedfull_baseline', 'combinedfull_truth_history', 45.92)}


def load(n):
    return json.loads((HERE / 'replay' / f'{n}.json').read_bytes())['rows']


def green_start(name):
    with gzip.open(HERE / f'extract_{name}.json.gz', 'rt', encoding='utf-8') as f:
        rows = json.load(f)['trajectory']
    return next(r['time'] for r in rows if r['light'] == 'green'), rows[-1]['time']


out = {}
for name, (base, cand, limit) in PAIRS.items():
    a, b = load(base), load(cand)
    start, end = green_start(name)
    end = limit or end
    counts = collections.Counter()
    zero_to_pos, pos_to_zero = [], []
    for x, y in zip(a, b):
        if not start + 1. <= x['t'] <= end or x['rec_v'] is None:
            continue
        counts['calls'] += 1
        counts['fresh_scans'] += x['fresh_acc'] is not None
        counts['fresh_rejected_base'] += x['fresh_acc'] is False
        counts['fresh_rejected_truth'] += y['fresh_acc'] is False
        counts['zero_base'] += x['v'] == 0
        counts['zero_truth'] += y['v'] == 0
        counts['steer_gt_0p02'] += abs(x['s'] - y['s']) > .02
        if x['v'] == 0 and y['v'] > 0:
            zero_to_pos.append((round(x['t'], 2), x['reason'], y['reason']))
        if x['v'] > 0 and y['v'] == 0:
            pos_to_zero.append((round(x['t'], 2), x['reason'], y['reason']))
    out[name] = dict(counts=dict(counts), base_zero_became_positive=len(zero_to_pos), base_positive_became_zero=len(pos_to_zero),
                     examples_zero_to_positive=zero_to_pos[:12], examples_positive_to_zero=pos_to_zero[:12],
                     zero_reasons_base=collections.Counter(x['reason'] for x in a if x['v'] == 0 and start + 1. <= x['t'] <= end).most_common(8),
                     zero_reasons_truth=collections.Counter(x['reason'] for x in b if x['v'] == 0 and start + 1. <= x['t'] <= end).most_common(8))
    print(name, json.dumps(out[name], ensure_ascii=False)[:1400])
with (HERE / 'replay_summary.json').open('w', encoding='utf-8', newline='\n') as f:
    json.dump(dict(scope='기록 궤적 고정 개방 루프. 판단 민감도일 뿐 랩 시간 효과가 아니다. 결합 실행은 경계 상실 시각까지만.', runs=out), f, ensure_ascii=False, indent=1)
