"""같은 저장 입력의 장착 변동 shadow 재생. 개루프 반사실이며 폐루프 주행 결과가 아니다.

사용: shadow_replay.py <실행 이름> <jr|c> → shadow_<실행 이름>.json
- base: 기록 그대로(재생 일치 확인)
- calib_dx: 실제 센서는 그대로, 제어기 lidar_x_m만 ±5/10 mm 틀린 경우(보정 오차)
- mask: 지주/센서 위치·지름·여유 변동의 차폐 각도. 기록은 명목 각도가 이미 NaN이므로
  명목∪변동만 표현할 수 있다(더 작은 Ø4 등은 명목에 포함되어 명목이 보수 상계).
- raw: 명목 지주 각도의 광선에 지주까지 거리를 넣고 구조 마스크를 뺀다(자기반환 그대로 처리).
판단 단계(follow)만 재생한다. 독립 보호층은 같은 스캔으로 전방 관측 판정과
정지 상태 스윕 간격만 따로 계산한다. 첫 분기 뒤 입력은 실제 변형 주행과 다르다.
"""
import copy
import json
import math
from multiprocessing import Pool
from pathlib import Path
import sys

import numpy as np

from arena_autonomy.follow_trace import freeze, replay_events, thaw
from arena_autonomy.lidar_observation import audit_front_observation
from arena_autonomy.local_path import swept_limit

HERE = Path(__file__).resolve().parent
MARKER = (-0.1005, 0., 0.05)
LAYOUTS = {
    'jr': dict(sx=.050, posts=[(.010, .062), (.010, -.062), (-.080, .052), (-.080, -.052)]),
    'c': dict(sx=.030, posts=[(.080, .062), (.080, -.062), (-.080, .062), (-.080, -.062)]),
}


def sectors(sx, posts, diameter=.007, margin=.5, marker=True):
    out = []
    for x, y in posts:
        r = math.hypot(x-sx, y)
        out.append([math.degrees(math.atan2(y, x-sx)), math.degrees(math.asin(diameter/2/r))+margin])
    if marker:
        mx, _, w = MARKER
        first, second = math.atan2(w/2, mx-sx), math.atan2(-w/2, mx-sx)
        diff = math.remainder(first-second, math.tau)
        out.append([math.degrees(math.remainder(second+diff/2, math.tau)), math.degrees(abs(diff)/2)+margin])
    return out


def in_sectors(angles, spec):
    hit = np.zeros(len(angles), dtype=bool)
    for center, width in spec:
        offset = np.remainder(angles-math.radians(center)+math.pi, math.tau)-math.pi
        hit |= np.abs(offset) <= math.radians(width)+1e-12
    return hit


def post_ranges(angles, sx, posts, diameter):
    """센서 원점에서 각 광선이 Ø 원기둥에 처음 닿는 거리(없으면 NaN)."""
    out = np.full(len(angles), np.nan)
    d = np.column_stack((np.cos(angles), np.sin(angles)))
    for x, y in posts:
        c = np.array([x-sx, y])
        b = d@c
        disc = b*b-(c@c-(diameter/2)**2)
        ok = disc >= 0
        t = np.where(ok, b-np.sqrt(np.where(ok, disc, 0.)), np.nan)
        out = np.where(np.isnan(out) | (t < out), t, out)
    return out


def transform(events, *, extra=None, raw=None, dx=0.):
    result = []
    for event in events:
        event = copy.deepcopy(event)
        if event['kind'] == 'header' and dx:
            event['data']['motion_config']['lidar_x_m'] += dx
        if event['kind'] == 'call' and (extra or raw):
            data = event['data']['input']
            scan = data['scan']
            ranges = np.asarray(thaw(scan['ranges']), dtype=float)
            angles = scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
            spec = data['rear_half_angle_deg']
            if not isinstance(spec, dict):
                spec = {'rear_half_angle_deg': spec, 'sectors_deg': []}
            if extra:
                ranges[in_sectors(angles, extra)] = np.nan
                spec = {**spec, 'sectors_deg': [*spec['sectors_deg'], *extra]}
            if raw:
                sx, posts, diameter = raw
                values = post_ranges(angles, sx, posts, diameter)
                hit = np.isfinite(values) & (values >= scan['range_min'])
                ranges[hit] = values[hit]
                # 지주 각도 마스크만 뺀다. 마커 등 다른 구조 각도는 유지한다.
                keep = [s for s in spec['sectors_deg'] if not any(
                    abs(math.remainder(math.radians(s[0])-math.atan2(y, x-sx), math.tau)) < 1e-6 for x, y in posts)]
                spec = {**spec, 'sectors_deg': keep}
            scan['ranges'] = freeze(ranges.tolist())
            data['rear_half_angle_deg'] = spec if spec['sectors_deg'] or spec['rear_half_angle_deg'] else spec['rear_half_angle_deg']
        result.append(event)
    return result


def guard_view(events, motion_x):
    """같은 스캔에서 독립 보호층의 전방 관측 판정과 정지 상태(현재 조향) 스윕 간격."""
    reasons, zero_clear = {}, 0
    for event in events:
        if event['kind'] != 'call':
            continue
        data = event['data']['input']
        scan = data['scan']
        ranges = np.asarray(thaw(scan['ranges']), dtype=float)
        audit = audit_front_observation(ranges, scan['angle_min'], scan['angle_increment'],
                                        scan['range_min'], scan['range_max'])
        reasons[audit.reason] = reasons.get(audit.reason, 0)+1
        valid = np.isfinite(ranges) & (ranges >= scan['range_min']) & (ranges <= scan['range_max'])
        angles = scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
        points = np.column_stack((ranges[valid]*np.cos(angles[valid])+motion_x, ranges[valid]*np.sin(angles[valid])))
        if swept_limit(points, data['previous_steering_rad']) <= 0.:
            zero_clear += 1
    return dict(front_audit=reasons, zero_swept_clearance_calls=zero_clear)


def summarize(name, result, base):
    out = result['outcomes']
    speeds = np.array([o['speed_mps'] for o in out])
    steer = np.array([o['steering_rad'] for o in out])
    summary = dict(variant=name, calls=result['calls'], matched_recorded=result['matched'],
                   mean_speed_mps=float(speeds.mean()), zero_calls=int(np.sum(speeds <= .001)),
                   behaviors={})
    for o in out:
        key = o['details'].get('behavior') or o['details'].get('reason')
        summary['behaviors'][key] = summary['behaviors'].get(key, 0)+1
    if base is not None:
        bs = np.array([o['speed_mps'] for o in base['outcomes']])
        bt = np.array([o['steering_rad'] for o in base['outcomes']])
        diff = np.abs(speeds-bs) > 1e-9
        diff |= np.abs(steer-bt) > 1e-9
        first = int(np.argmax(diff)) if diff.any() else None
        summary.update(changed_vs_base=int(diff.sum()),
                       first_change_reference_s=None if first is None else out[first]['reference_s'],
                       max_abs_speed_delta=float(np.max(np.abs(speeds-bs))),
                       max_abs_steer_delta=float(np.max(np.abs(steer-bt))))
    return summary


EVENTS = None


def load(path):
    global EVENTS
    with Path(path).open(encoding='utf-8') as stream:
        EVENTS = [json.loads(line) for line in stream]


def job(args):
    name, kwargs = args
    variant = transform(EVENTS, **kwargs) if kwargs else EVENTS
    result = replay_events(variant)
    motion_x = EVENTS[0]['data']['motion_config']['lidar_x_m']
    guard = guard_view(variant, motion_x+kwargs.get('dx', 0.))
    return name, result, guard


def variants(layout):
    sx, posts = layout['sx'], layout['posts']
    yield 'base', {}
    for dx in (-.010, -.005, .005, .010):
        yield f'calib_lidar_x_{dx*1000:+.0f}mm', dict(dx=dx)
    for dsx in (-.010, -.005, .005, .010):
        yield f'sensor_x_{dsx*1000:+.0f}mm', dict(extra=sectors(sx+dsx, posts))
    for d in (.008, .010):
        yield f'diameter_{d*1000:.0f}mm', dict(extra=sectors(sx, posts, diameter=d))
    for group, rows in (('front', [0, 1]), ('rear', [2, 3])):
        for ddx in (-.010, -.005, .005, .010):
            moved = [(x+ddx, y) if i in rows else (x, y) for i, (x, y) in enumerate(posts)]
            yield f'{group}_posts_x_{ddx*1000:+.0f}mm', dict(extra=sectors(sx, moved))
        for ddy in (-.005, .005):
            moved = [(x, y+math.copysign(ddy, y)) if i in rows else (x, y) for i, (x, y) in enumerate(posts)]
            yield f'{group}_posts_halfwidth_{ddy*1000:+.0f}mm', dict(extra=sectors(sx, moved))
    for margin in (1.0, 1.5):
        yield f'margin_{margin:.1f}deg', dict(extra=sectors(sx, posts, margin=margin))
    yield 'raw_self_return_d7', dict(raw=(sx, posts, .007))
    yield 'raw_self_return_d4', dict(raw=(sx, posts, .004))


def main(run_name, layout_name, workers=12):
    run = HERE/run_name
    layout = LAYOUTS[layout_name]
    recorded = json.loads((run/'lidar_blind_sectors.json').read_text()) if (run/'lidar_blind_sectors.json').exists() else None
    expected = sectors(layout['sx'], layout['posts'])
    nominal_ok = recorded is not None and len(recorded) == len(expected) and all(
        math.isclose(a, b, abs_tol=1e-9) for r, e in zip(recorded, expected) for a, b in zip(r, e))
    jobs = list(variants(layout))
    with Pool(min(int(workers), len(jobs)), initializer=load, initargs=(str(run/'follow_decisions.jsonl'),)) as pool:
        results = pool.map(job, jobs)
    base = next(r for n, r, _ in results if n == 'base')
    rows = []
    for name, result, guard in results:
        row = summarize(name, result, None if name == 'base' else base)
        row['guard_same_scan'] = guard
        rows.append(row)
    report = dict(run=run_name, layout=layout_name, nominal_sectors_match_runtime=nominal_ok,
                  nominal_sectors_deg=expected, base_replay_matches_recording=base['passed'],
                  scope='개루프 같은 입력 재생. 첫 분기 뒤 판단은 실제 변형 주행이 아니다. 보호층 값은 같은 스캔의 '
                        '전방 관측 판정·현재 조향 스윕 간격만이며 보호층 전체 재생이 아니다. 명목∪변동 마스크만 표현 가능.',
                  variants=rows)
    (HERE/f'shadow_{run_name}.json').write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding='utf-8', newline='\n')
    print(json.dumps([{k: r.get(k) for k in ('variant', 'changed_vs_base', 'first_change_reference_s', 'mean_speed_mps', 'zero_calls')}
                      for r in rows], ensure_ascii=False))
    return 0 if base['passed'] and nominal_ok else 1


if __name__ == '__main__':
    raise SystemExit(main(*sys.argv[1:4]))
