"""계획·보호·MCU·모터축 환산·차체의 기록된 시각별 대조. 평가 전용."""
from bisect import bisect_right
from collections import Counter
import json
from pathlib import Path
import statistics
import sys

HERE = Path(__file__).resolve().parent


def analyze(name):
    run = HERE.parent/'official_forward_v1'/name
    report = json.loads((run/'report.json').read_bytes())
    green = report['evaluation']['t_green']
    trace = json.loads((run/'trace.json').read_bytes())
    mcu = json.loads((run/'actuation_trace.json').read_bytes())
    stamps = [r['now_us']*1e-6 for r in mcu]
    assert all(a <= b for a, b in zip(stamps, stamps[1:]))
    rows = []
    for row in trace:
        if row['t'] < green:
            continue
        i = bisect_right(stamps, row['t'])-1
        data = mcu[i] if i >= 0 and 0 <= row['t']-stamps[i] < .1 else None
        rows.append(dict(t=row['t']-green, plan=row.get('drive_speed_mps'), safe=row['safe_speed_mps'],
                         mcu=None if data is None else data['target_speed_mps'],
                         shaft=None if data is None or not data['feedback_valid'] else data['mean_wheel_mps'],
                         body=row['ground_speed_mps'], behavior=row['autonomy'].get('behavior'),
                         feedback_mode=None if data is None else data.get('feedback_mode'),
                         mcu_state=None if data is None else data['state']))
    bins = []
    for edge in range(int(max(r['t'] for r in rows))+1):
        group = [r for r in rows if edge <= r['t'] < edge+1]
        if not group:
            continue
        avg = lambda k: statistics.mean(r[k] for r in group if r[k] is not None) if any(r[k] is not None for r in group) else None
        bins.append(dict(t_s=edge, means_mps={k: avg(k) for k in ('plan','safe','mcu','shaft','body')},
                         mcu_missing=sum(r['mcu'] is None for r in group),
                         behaviors=dict(Counter(r['behavior'] for r in group))))
    result = dict(scope='recorded data only; motor shaft conversion is not measured body velocity',
                  clocks='trace sim seconds matched to virtual MCU now_us in the same simulator; latest <=100 ms',
                  run=name, run_passed=report['passed'], feedback_modes=sorted({r['feedback_mode'] for r in rows if r['feedback_mode']}),
                  timeline=bins)
    output = HERE/f'{name}_actuation_compare.json'
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))


if __name__ == '__main__':
    analyze(sys.argv[1])
