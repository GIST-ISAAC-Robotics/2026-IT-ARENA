"""부모 노드 증속 제한의 재시작 기준 비교(평가 전용, 개루프). replay.py 뒤에 실행.

현재: v = min(계획, sqrt(3/곡률), 직전 발행 + 1.5·dt)  — wall_follow.py control()
후보 P: 직전 발행 대신 max(직전 발행, 측정 평균 바퀴 속도)에서 증속한다. 계획 0(HOLD)은 그대로 0이다.
후보 R: 0.2 s 이하 HOLD 직후 첫 양수 호출에서만 min(측정 속도, HOLD 직전 발행)에서 다시 증속한다.
입력: replay.json의 재생 출력(현재/후보 A)과 run1 저장 입력의 측정 속도(speed_mps). 차체 응답·이후 입력 변화는 반영하지 않는다.
"""
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUN = HERE.parent/'run1'
ACCEL, DT, LAT, WB = 1.5, .02, 3.0, .135


def calls():
    out = []
    with (RUN/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        next(stream)
        for line in stream:
            e = json.loads(line)
            if e['kind'] == 'call':
                v, steer, d = e['data']['output']
                out.append(dict(t=e['data']['input']['reference_s'], out=v, steer=steer,
                                measured=e['data']['input']['speed_mps'], reason=d.get('reason')))
    return out


def publish(rows, measured_base):
    last, series = 0., []
    for r in rows:
        v = r['out']
        k = abs(math.tan(r['steer'])/WB)
        if k > 1e-6:
            v = min(v, math.sqrt(LAT/k))
        base = max(last, r['measured']) if measured_base and math.isfinite(r['measured']) and r['measured'] > 0 else last
        v = min(v, base+ACCEL*DT)
        series.append(v)
        last = v
    return series


def publish_short_hold(rows, max_calls=10):
    """후보 R: 0.2 s 이하의 짧은 계획 0(HOLD) 직후 첫 양수 호출에서만, 증속 기준을
    min(측정 속도, HOLD 직전 발행값)으로 되살린다. 그 밖의 호출·HOLD 중 0·긴 정지는 현재와 같다."""
    last, series, streak, before = 0., [], 0, 0.
    for r in rows:
        v = r['out']
        k = abs(math.tan(r['steer'])/WB)
        if k > 1e-6:
            v = min(v, math.sqrt(LAT/k))
        base = last
        if v > 0. and 0 < streak <= max_calls and math.isfinite(r['measured']) and r['measured'] > 0:
            base = max(last, min(r['measured'], before))
        if r['out'] <= 0.:
            if streak == 0:
                before = last
            streak += 1
        else:
            streak = 0
        v = min(v, base+ACCEL*DT)
        series.append(v)
        last = v
    return series


def main():
    rows = calls()
    replay = json.loads((HERE/'replay.json').read_bytes())
    changed = {round(c['t'], 3): c for c in replay['candidate_A']['changed']}
    rows_a = [dict(r, out=changed[round(r['t'], 3)]['candidate']) if round(r['t'], 3) in changed else r for r in rows]
    t = [r['t'] for r in rows]
    variants = {'current': publish(rows, False), 'A_bridge_release': publish(rows_a, False),
                'P_measured_ramp_base': publish(rows, True), 'A_plus_P': publish(rows_a, True),
                'R_short_hold_resume': publish_short_hold(rows), 'A_plus_R': publish_short_hold(rows_a)}

    def gain(name, lo, hi):
        return round(sum((b-a)*DT for tt, a, b in zip(t, variants['current'], variants[name]) if lo <= tt <= hi), 4)

    def changed_calls(name, lo, hi):
        return [round(tt, 2) for tt, a, b in zip(t, variants['current'], variants[name]) if lo <= tt <= hi and abs(a-b) > 1e-9]
    result = dict(scope=__doc__.strip(), model=dict(acceleration_mps2=ACCEL, dt_s=DT, lateral_acceleration_mps2=LAT, wheelbase_m=WB))
    for name in variants:
        if name == 'current':
            continue
        lap = changed_calls(name, 9., 1e9)
        result[name] = dict(window_37_40p2_command_distance_gain_m=gain(name, 37., 40.2),
                            lap_command_distance_gain_m=gain(name, 9., 1e9),
                            changed_published_calls_lap=len(lap),
                            changed_outside_window=[x for x in lap if not 37. <= x <= 40.2][:30])
    # 계획이 양수인데 현재 부모 발행이 그보다 0.01 m/s 이상 낮은 시간(증속 제한·횡가속 제한이 정한 시간).
    result['current_parent_limited_s'] = {
        f'{lo:g}-{hi:g}s': round(sum(DT for tt, r, p in zip(t, rows, variants['current'])
                                     if lo <= tt <= hi and r['out'] > .001 and p < r['out']-.01), 2)
        for lo, hi in ((37., 38.38), (38.38, 39.9), (39.9, 40.2))}
    result['window_series'] = [dict(t=round(tt, 2), planner=round(r['out'], 3), measured=round(r['measured'], 3),
                                    **{k: round(v[i], 3) for k, v in variants.items()})
                               for i, (tt, r) in enumerate(zip(t, rows)) if 38.3 <= tt <= 39.9]
    (HERE/'parent_model.json').write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding='utf-8', newline='\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('window_series', 'scope')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
