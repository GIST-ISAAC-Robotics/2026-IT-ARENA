"""두 실행의 로그로 입력 나이·판단 주기·보호 제한을 나란히 센다. 별도 성능 시험이나 Jetson 근거가 아니다."""
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path

import numpy as np

HERE=Path(__file__).resolve().parent


def percentile(values):
    values=np.asarray(values,dtype=float)
    return dict(count=int(len(values)),p50=float(np.percentile(values,50)),p95=float(np.percentile(values,95)),max=float(np.max(values)))


def summary(folder):
    m=json.loads((folder/'run1_metrics.json').read_bytes())
    begin=m['first_green_sample_s'];end=begin+m['lap_s_from_first_green_sample']
    report=json.loads((folder/'run1/report.json').read_bytes())
    times=[];ages=[];reasons=Counter()
    for line in (folder/'run1/follow_decisions.jsonl').open(encoding='utf-8'):
        e=json.loads(line)
        if e['kind']!='call':continue
        i=e['data']['input'];o=e['data']['output'][2]
        if begin<=i['reference_s']<=end:
            times.append(i['reference_s'])
            if o.get('source_first_s') is not None:ages.append(i['reference_s']-o['source_first_s'])
            reasons[o.get('reason')]+=1
    steps=np.diff(times)
    rows=[r for r in map(json.loads,(folder/'run1/trajectory.jsonl').read_bytes().splitlines()) if r['light']=='green' and begin<=r['time']<=end]
    processing=[r['autonomy']['motion']['processing_ms'] for r in rows if isinstance(r['autonomy'].get('motion'),dict) and 'processing_ms' in r['autonomy']['motion']]
    wheel=[r['autonomy']['motion']['wheel_age_s'] for r in rows if isinstance(r['autonomy'].get('motion'),dict) and 'wheel_age_s' in r['autonomy']['motion']]
    safety=Counter(r['safety']['reason'] for r in rows if isinstance(r.get('safety'),dict))
    capped=[r['autonomy']['speed_command_mps']-r['safe_drive_speed_mps'] for r in rows
            if isinstance(r.get('safe_drive_speed_mps'),(int,float)) and isinstance(r['autonomy'].get('speed_command_mps'),(int,float))]
    watchdog=sum(bool(r['dynamics'].get('watchdog_stop')) for r in rows)
    return dict(lap_s=m['lap_s_from_first_green_sample'],decisions=len(times),decision_step_s=percentile(steps),
        decision_steps_over_30ms=int(np.sum(steps>.03)),scan_age_s=percentile(ages),lidar_motion_processing_ms=percentile(processing),
        wheel_age_s=percentile(wheel),safety_reason_samples=dict(safety),planner_minus_safe_mps=percentile(capped),
        watchdog_stop_samples=int(watchdog),drive_delivery=report.get('drive_delivery'),sim_safety_reasons=m.get('safety_reasons'))


result=dict(scope='기록된 값의 나란한 비교. 판단 함수 자체의 온라인 소요 시간은 기록에 없고 재생 비용은 compare.json의 PC 값이다.',
    baseline=summary(HERE.parent/'solo_preview_gate_v1'),trial=summary(HERE),script_sha256=sha256(Path(__file__).read_bytes()).hexdigest())
(HERE/'online_cost.json').write_text(json.dumps(result,ensure_ascii=False,indent=1),encoding='utf-8',newline='\n')
print(json.dumps(result,ensure_ascii=False,indent=0))
