"""11/4 Ø2 mm JR60P20 기록(다른 코드)에서 같은 43.4 m 0명령이 후보 A 조건에 해당하는지 기록값으로만 확인.

재생이 아니다(당시 코드가 달라 현재 사본으로 재생하지 않는다). 각 0명령 호출의 기록 세부값으로 조건을 판정하고,
조건이 맞는 호출을 기록 target_speed_before_hold로 바꿨을 때의 부모 증속 모델 명령 적분 차이를 계산한다(개루프).
"""
import json
from pathlib import Path

from parent_model import publish, DT

HERE = Path(__file__).resolve().parent
PRIOR = HERE.parents[2]/'2026-11-04/lidar_jr_forward_v1/jr60p20_1'
RESUME = .28  # 당시 follow_config.resume_clearance_m(헤더에서 확인)


def main():
    rows, hits = [], []
    bridged = False
    with (PRIOR/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        header = json.loads(next(stream))['data']['follow_config']
        assert header['resume_clearance_m'] == RESUME
        for line in stream:
            e = json.loads(line)
            if e['kind'] != 'call':
                continue
            v, steer, d = e['data']['output']
            t = e['data']['input']['reference_s']
            row = dict(t=t, out=v, steer=steer, measured=e['data']['input']['speed_mps'])
            # 기록에서 보이는 조건: 직전 호출이 회랑 다리 양수, 이번이 재개 대기, 쓸림 여유·기본 목표 충족.
            # (hold 사유·재개 확인 횟수는 기록에 직접 없어 직전 다리의 underlying_reason과 fresh 수락으로 대신 확인)
            if (v == 0. and bridged and d.get('reason') == 'follow_release_wait'
                    and (d.get('swept_clearance_m') or 0.) >= RESUME and (d.get('target_speed_before_hold_mps') or 0.) > 0.):
                hits.append(dict(t=round(t, 3), before_hold=d['target_speed_before_hold_mps'], clearance=d['swept_clearance_m']))
                row['candidate'] = d['target_speed_before_hold_mps']
            positive = row.get('candidate', v) > 0.
            bridged = positive and (d.get('reason') == 'local_follow_corridor_candidate' or 'candidate' in row)
            rows.append(row)
    cand = [dict(r, out=r.get('candidate', r['out'])) for r in rows]
    a, b = publish(rows, False), publish(cand, False)
    gain = sum((y-x)*DT for r, x, y in zip(rows, a, b) if 37. <= r['t'] <= 41.)
    result = dict(scope=__doc__.strip(), run=str(PRIOR.relative_to(HERE.parents[4])), condition_hits=hits,
                  command_distance_gain_37_41_m=round(gain, 4))
    (HERE/'prior_check.json').write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding='utf-8', newline='\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'scope'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
