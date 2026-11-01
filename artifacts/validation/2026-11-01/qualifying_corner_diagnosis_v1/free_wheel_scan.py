"""좌우 바퀴 표면 속도와 평면 운동학 기대값의 차이로 헛돎 구간을 전 구간 집계한다.

기대값: v_L/R = vx -/+ yaw_rate*T/2 (T=윤거 0.135 m). 평가 정답을 쓰는 사후 판정이다.
접촉력은 기록되지 않았으므로 '바퀴 속도가 차체 운동에서 벗어난 구간'만 직접 사실이다.
"""
import gzip
import json
import statistics
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
R, TRACK = .0325, .135
THRESH = .20        # 표본 간격 약 46 ms에서 한 칸 잡음이 아닌 뚜렷한 이탈만 센다
SOURCES = {
    'rise_only': (HERE / 'extract_rise_only.json.gz', None),
    'turn_only': (HERE / 'extract_turn_only.json.gz', None),
    'combined_failed': (HERE / 'extract_combined_failed.json.gz', None),
    'solo_2p0': (None, ROOT / 'corner_approach_v1/solo_2p0/trajectory.jsonl'),
    'solo_10kmh': (None, ROOT / 'corner_approach_v1/solo_10kmh/trajectory.jsonl'),
}


def rows_from_raw(path):
    out = []
    with path.open(encoding='utf-8') as f:
        for line in f:
            r = json.loads(line)
            d = r.get('dynamics') or {}
            if not d or d.get('left_speed_rad_s') is None:
                continue
            out.append(dict(time=r['time'], progress_m=r['progress_m'], light=r['light'], x=r['x'], y=r['y'],
                            truth_longitudinal_mps=d['truth_longitudinal_mps'],
                            truth_yaw_rate_rad_s=d['truth_yaw_rate_rad_s'], left_speed_rad_s=d['left_speed_rad_s'],
                            right_speed_rad_s=d['right_speed_rad_s'], motor_torque_nm=d['motor_torque_nm'],
                            applied_target_mps=d['applied_target_mps']))
    return out


def load(name):
    gz, raw = SOURCES[name]
    if gz:
        with gzip.open(gz, 'rt', encoding='utf-8') as f:
            rows = json.load(f)['trajectory']
        return [r for r in rows if r.get('left_speed_rad_s') is not None]
    return rows_from_raw(raw)


def episodes(rows):
    eps, cur = [], None
    for i, r in enumerate(rows):
        vx, w = r['truth_longitudinal_mps'], r['truth_yaw_rate_rad_s']
        el = r['left_speed_rad_s'] * R - (vx - w * TRACK / 2)
        er = r['right_speed_rad_s'] * R - (vx + w * TRACK / 2)
        side = 'L' if abs(el) >= abs(er) else 'R'
        e = el if side == 'L' else er
        hit = abs(e) > THRESH and vx > .25
        if hit and cur and cur['side'] == side and i - cur['last'] <= 1:
            cur['last'] = i
            cur['n'] += 1
            cur['t1'] = r['time']
            cur['p1'] = r['progress_m']
            cur['max_err'] = max(cur['max_err'], abs(e), key=abs)
            cur['tq'].append(r['motor_torque_nm'])
            cur['v1'] = vx
        elif hit:
            if cur:
                eps.append(cur)
            cur = dict(side=side, last=i, n=1, t0=r['time'], t1=r['time'], p0=r['progress_m'], p1=r['progress_m'],
                       max_err=abs(e), tq=[r['motor_torque_nm']], v0=vx, v1=vx, tgt0=r['applied_target_mps'])
    if cur:
        eps.append(cur)
    return eps


out = {}
for name in SOURCES:
    rows = [r for r in load(name) if r['light'] == 'green']
    eps = episodes(rows)
    moving = [r for r in rows if r['truth_longitudinal_mps'] > .25]
    longest = sorted(eps, key=lambda e: -e['n'])[:6]
    out[name] = dict(
        moving_samples=len(moving), episodes=len(eps), single_sample=sum(1 for e in eps if e['n'] == 1),
        samples_in_episode=sum(e['n'] for e in eps), ge_3_samples=sum(1 for e in eps if e['n'] >= 3),
        ge_6_samples=sum(1 for e in eps if e['n'] >= 6),
        longest=[dict(side=e['side'], n=e['n'], t0=round(e['t0'], 3), t1=round(e['t1'], 3), p0=round(e['p0'], 2),
                      p1=round(e['p1'], 2), max_err=round(e['max_err'], 2), v0=round(e['v0'], 2), v1=round(e['v1'], 2),
                      mean_abs_torque=round(statistics.mean(abs(x) for x in e['tq']), 4)) for e in longest])
with (HERE / 'free_wheel_episodes.json').open('w', encoding='utf-8', newline='\n') as f:
    json.dump(dict(threshold_mps=THRESH, min_body_speed_mps=.25,
                   scope='녹색 이후 전체 시험 표본. 정답 기반 사후 판정이며 접촉력 직접 측정이 아니다.', runs=out), f, ensure_ascii=False, indent=1)
for k, v in out.items():
    print(k, {x: y for x, y in v.items() if x != 'longest'})
    for e in v['longest'][:4]:
        print('   ', e)
