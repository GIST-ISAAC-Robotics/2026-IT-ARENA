"""저장된 JR60P20 구조 고속 단독 주행 1건(11/4 lidar_jr_forward_v1/jr60p20_1)의 판단 입력을 현재 코드로 재생.

r7 수정 대상(일반 결측이 있을 때 증속 후보)이 아닌 호출의 출력이 바뀌지 않았는지 본다. 출력: replay_representative.json
WSL: environment.sh로 ROS/설치 경로를 잡은 뒤 python3 -B replay_representative.py
"""
import hashlib
import json
from pathlib import Path

from arena_autonomy.follow_trace import replay_events

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
TRACE = ROOT/'artifacts/validation/2026-11-04/lidar_jr_forward_v1/jr60p20_1/follow_decisions.jsonl'


def main():
    with TRACE.open(encoding='utf-8') as stream:
        result = replay_events(json.loads(line) for line in stream)
    mismatches = result.get('mismatches', [])
    rows = []
    for m in mismatches:
        e, a = m['expected'], m['actual']
        ed, ad = e[2], a[2]
        diff = sorted(k for k in set(ed) | set(ad) if ed.get(k) != ad.get(k))
        po = ed.get('partial_observation') or {}
        rows.append(dict(seq=m['seq'], command_equal=e[0] == a[0] and e[1] == a[1], expected_v=e[0], actual_v=a[0],
                         differing_detail_keys=diff, recorded_partial_reason=po.get('reason'),
                         recorded_partial_cap=po.get('speed_cap_mps'), recorded_open_route=ed.get('open_route_speed_enabled'),
                         recorded_target_mps=ed.get('target_speed_before_hold_mps')))
    out = dict(classified=rows, all_commands_equal=all(r['command_equal'] for r in rows),
               differing_keys_union=sorted({k for r in rows for k in r['differing_detail_keys']}),
               trace=TRACE.relative_to(ROOT).as_posix(), trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),
               local_follow_sha256=hashlib.sha256((ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py').read_bytes()).hexdigest(),
               calls=result.get('calls'), mismatch_count=len(mismatches), first_mismatches=mismatches[:5],
               keys=sorted(result))
    (HERE/'replay_representative.json').write_text(json.dumps(out, indent=1, ensure_ascii=False, default=str) + '\n',
                                                   encoding='utf-8', newline='\n')
    print(json.dumps({k: out[k] for k in ("calls", "mismatch_count", "all_commands_equal", "differing_keys_union")}))


if __name__ == '__main__':
    main()
