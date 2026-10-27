"""run1 동일 재생 중 GUARD에서 평행 후보가 정지를 낸 판단에 기존 RELEASED 이동 검사를 적용해 보는 설명용 반사실.

제어 출력은 바꾸지 않는다(재생 일치 확인). 기존 G1이 정상 양수일 때만, 그 G1 출력의 반응+제동 구간 차체 모서리 횡변위가
1 cm 이하인지, 아니면 차체·바퀴 외곽 이동 영역이 관측/작은 가림 꼬리 조건을 만족하는지를 기록한다. 채택 제안이 아니다.
"""
import json
from collections import Counter
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
for part in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts'):
    sys.path.insert(0, str(ROOT/part))

import arena_autonomy.pass_release_trial as ps  # noqa: E402
from arena_autonomy.follow_trace import replay_events  # noqa: E402

RUN = HERE/'run1_corner'
original = ps.PassReleaseSelector.select
rows = []


def instrumented(self, scene, baseline, parallel, speed, previous, dt):
    out = original(self, scene, baseline, parallel, speed, previous, dt)
    d = out[2]
    b_speed, b_steer, b_detail = baseline
    if (d.get('selector_mode') == 'GUARD' and d.get('selector_choice') == 'parallel' and out[0] <= 0
            and b_detail.get('reason') in ps.NORMAL_BASELINE and b_speed > 0 and speed >= 0):
        age = scene.t_ref_s-scene.source_first_s
        excursion = ps.lateral_excursion(max(speed, b_speed), age, b_steer, previous, dt, self.follow_config)
        verdict, region = 'small_motion', None
        if excursion > self.config.small_excursion_m:
            region = ps.motion_region(scene, speed, b_speed, b_steer, previous, dt, self.follow_config, self.lat)
            verdict = 'observed_large_motion'
            if region['unobserved']:
                verdict = 'unobserved'
            elif (region['masked_left'] or region['masked_right']) and (
                    region['masked_front_samples'] > 0 or region['masked_tail_extent_m'] > self.config.masked_tail_allowance_m):
                verdict = 'masked_beyond_allowance'
        rows.append({'t': scene.t_ref_s, 'parallel_reason': (d.get('parallel_hold_trial') or {}).get('reason'),
                     'baseline_speed': b_speed, 'baseline_steering': b_steer, 'excursion_m': excursion,
                     'verdict': verdict, 'region': region})
    return out


ps.PassReleaseSelector.select = instrumented
with (RUN/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
    report = replay_events(json.loads(line) for line in stream)
ps.PassReleaseSelector.select = original
summary = {'scope': 'explanatory counterfactual on identical replay; control outputs unchanged; not an adopted rule',
           'replay': {k: report[k] for k in ('calls', 'matched', 'passed')},
           'guard_parallel_stop_with_positive_baseline': len(rows),
           'time_range_s': [rows[0]['t'], rows[-1]['t']] if rows else None,
           'parallel_reasons': dict(Counter(r['parallel_reason'] for r in rows)),
           'released_style_verdicts': dict(Counter(r['verdict'] for r in rows)),
           'excursion_m_range': [min(r['excursion_m'] for r in rows), max(r['excursion_m'] for r in rows)] if rows else None}
(HERE/'run1_guard_counterfactual.json').write_text(json.dumps({'summary': summary, 'rows': rows}, ensure_ascii=False,
                                                              indent=1)+'\n', encoding='utf-8', newline='\n')
print(json.dumps(summary, ensure_ascii=False))
