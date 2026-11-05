"""구조 각도 검산 + 11/5 기준 주행 톱니 원인 분리(저장 입력만). 출력: checks.json

python -I -B checks.py  (표준 라이브러리만)
"""
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RUN = ROOT / 'artifacts/validation/2026-11-05/moving_pass_baseline_v1/run1'

# 1. JR60P20 프리셋 좌표로 센서 원점 기준 구조 각도를 다시 계산해 런타임 파일과 대조.
lidar = (0.060, 0.0)
posts = [(0.020, 0.062, 0.007), (0.020, -0.062, 0.007), (-0.080, 0.052, 0.007), (-0.080, -0.052, 0.007)]
wires = [(-0.088906, 0.055308, 0.020), (-0.088906, -0.055308, 0.020)]
margin = 0.5
expected = []
for x, y, d in posts + wires:
    dx, dy = x - lidar[0], y - lidar[1]
    expected.append([math.degrees(math.atan2(dy, dx)), math.degrees(math.asin(d / 2 / math.hypot(dx, dy))) + margin])
runtime = json.loads((RUN / 'lidar_blind_sectors.json').read_text(encoding='utf-8'))
sector_check = dict(expected=expected, runtime=runtime,
                    max_abs_diff_deg=max(abs(a - b) for e, r in zip(expected, runtime) for a, b in zip(e, r)),
                    nested=[[i, j] for i, (c1, h1) in enumerate(runtime) for j, (c2, h2) in enumerate(runtime)
                            if i != j and abs(c1 - c2) + h1 <= h2 + 1e-9],
                    note='nested=[안쪽, 바깥] 지주 각도가 배선 각도 안에 들어 있어 지주 행은 양옆이 항상 마스크 → '
                         'classify에서 unexplained로 보고되지만 광선은 배선 행으로 면제된다(진단 표시만 영향).')

# 2. 기준 주행 톱니: 구조 정책 제한과 추종 예약 상한을 분리.
calls = []
with (RUN / 'follow_decisions.jsonl').open(encoding='utf-8') as f:
    for line in f:
        r = json.loads(line)
        if r['kind'] == 'call':
            calls.append(r['data'])
tg, te = 7.401, 21.401
w = [c for c in calls if tg <= c['input']['reference_s'] <= te]
part, ks_cap, applied, equal_cap, normal_above = {}, 0, 0, 0, 0
for c in w:
    x = c['output'][2]
    po = x['normal']['partial_observation']
    part[po['reason']] = part.get(po['reason'], 0) + 1
    ks_cap += po['known_structure'].get('speed_cap_mps') is not None or po.get('speed_cap_mps') is not None
    fe = x['follow_entry_space']
    if fe.get('applied'):
        applied += 1
        equal_cap += abs(c['output'][0] - fe['cap_mps']) < 1e-9
        normal_above += (x['normal'].get('target_speed_before_hold_mps') or 0) > fe['cap_mps'] + 1e-9
sawtooth = dict(calls_in_window=len(w), partial_observation_reason_counts=part,
                structure_or_partial_cap_calls=ks_cap,
                follow_entry_space_applied_calls=applied, output_equals_entry_space_cap=equal_cap,
                normal_target_above_entry_space_cap=normal_above,
                conclusion='창 안 부분관측은 전부 fully_observed, 구조/부분관측 상한 0회. 최종 속도는 follow_entry_space 상한과 같다. '
                           '톱니는 구조 적응이 아니라 handoff 추종 예약(reserve)·front_patch 경로에서 나온다.')
out = dict(scope='저장 입력/런타임 파일만. 런타임 코드 무변경.', sector_check=sector_check, baseline_sawtooth=sawtooth)
(HERE / 'checks.json').write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n', encoding='utf-8', newline='\n')
print(json.dumps({'max_abs_diff_deg': sector_check['max_abs_diff_deg'], 'nested': sector_check['nested'],
                  **{k: v for k, v in sawtooth.items() if k != 'conclusion'}}, ensure_ascii=False))
