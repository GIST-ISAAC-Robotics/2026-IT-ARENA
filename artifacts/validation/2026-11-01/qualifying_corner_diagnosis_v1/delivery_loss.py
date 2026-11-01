"""목표 변화가 차체에 전달되지 않은 구간을 제동/구동으로 나눠 전 구간 집계한다.

제동 손실: 적용 목표가 차체보다 0.3 m/s 이상 낮은데 0.3 s 창 차체 감속이 0.5 m/s² 미만.
구동 손실: 적용 목표가 차체보다 0.15 m/s 이상 높은데 0.3 s 창 차체 가속이 0.3 m/s² 미만이고
          바퀴 환산 평균은 이미 목표 근처(차이 0.15 이내)여서 PI가 토크를 키울 이유가 없는 경우.
둘 다 정답 차체 속도를 쓰는 사후 평가이며 제어 입력이 아니다.
"""
import bisect
import gzip
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SOURCES = {'rise_only': HERE / 'extract_rise_only.json.gz', 'turn_only': HERE / 'extract_turn_only.json.gz',
           'combined_failed': HERE / 'extract_combined_failed.json.gz',
           'solo_2p0': ROOT / 'corner_approach_v1/solo_2p0/trajectory.jsonl',
           'solo_10kmh': ROOT / 'corner_approach_v1/solo_10kmh/trajectory.jsonl'}
WINDOW = .30


def load(path):
    if path.suffix == '.gz':
        with gzip.open(path, 'rt', encoding='utf-8') as f:
            rows = json.load(f)['trajectory']
        return [dict(t=r['time'], p=r['progress_m'], v=r['truth_longitudinal_mps'], w=r['wheel_surface_speed_mps'],
                     a=r['applied_target_mps'], q=r['motor_torque_nm'], light=r['light']) for r in rows
                if r.get('wheel_surface_speed_mps') is not None]
    out = []
    with path.open(encoding='utf-8') as f:
        for line in f:
            r = json.loads(line)
            d = r.get('dynamics') or {}
            if d.get('wheel_surface_speed_mps') is None:
                continue
            out.append(dict(t=r['time'], p=r['progress_m'], v=d['truth_longitudinal_mps'], w=d['wheel_surface_speed_mps'],
                            a=d['applied_target_mps'], q=d['motor_torque_nm'], light=r['light']))
    return out


def scan(rows, kind):
    times = [r['t'] for r in rows]
    flags = []
    for i, r in enumerate(rows):
        j = bisect.bisect_left(times, r['t'] + WINDOW)
        if j >= len(rows):
            flags.append(False)
            continue
        slope = (rows[j]['v'] - r['v']) / (rows[j]['t'] - r['t'])
        if kind == 'brake':
            flags.append(r['v'] > .5 and r['a'] < r['v'] - .3 and slope > -.5)
        else:
            flags.append(r['v'] > .3 and r['a'] > r['v'] + .15 and abs(r['w'] - r['a']) < .15 and slope < .3)
    spans = []
    start = None
    for i, f in enumerate(flags + [False]):
        if f and start is None:
            start = i
        if not f and start is not None:
            a, b = rows[start], rows[i - 1]
            dur = b['t'] - a['t'] + WINDOW
            if dur >= .40:
                spans.append(dict(t0=round(a['t'], 2), dur=round(dur, 2), p0=round(a['p'], 2), p1=round(rows[min(i + 6, len(rows) - 1)]['p'], 2),
                                  v0=round(a['v'], 2), v1=round(b['v'], 2), target0=round(a['a'], 2), target1=round(b['a'], 2)))
            start = None
    return spans


result = {}
for name, path in SOURCES.items():
    rows = [r for r in load(path) if r['light'] == 'green']
    brake, drive = scan(rows, 'brake'), scan(rows, 'drive')
    result[name] = dict(brake_loss=brake, drive_loss=drive,
                        drive_loss_total_s=round(sum(x['dur'] for x in drive), 2),
                        brake_loss_total_s=round(sum(x['dur'] for x in brake), 2))
    print(name, 'brake_loss_s', result[name]['brake_loss_total_s'], 'drive_loss_s', result[name]['drive_loss_total_s'])
    for x in brake:
        print('   B', x)
    for x in sorted(drive, key=lambda d: -d['dur'])[:5]:
        print('   D', x)
with (HERE / 'delivery_loss.json').open('w', encoding='utf-8', newline='\n') as f:
    json.dump(dict(window_s=WINDOW, scope='정답 기반 사후 평가. 임계값은 진단용 분류선이며 주행 판정선이 아니다.', runs=result), f, ensure_ascii=False, indent=1)
