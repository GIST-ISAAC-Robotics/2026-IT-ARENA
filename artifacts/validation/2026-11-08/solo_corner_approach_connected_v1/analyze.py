"""같은 새 코스 계측 시험의 r19 전후 대조. 저장 평가 정답만 사용한다."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

HERE=Path(__file__).resolve().parent
rows=lambda p: [json.loads(line) for line in p.read_bytes().splitlines() if line.strip()]


def review(root):
    folder=root/'run1'
    report=json.loads((folder/'report.json').read_bytes())
    trace=json.loads((folder/'trace.json').read_bytes())
    surface=json.loads((root/'surface.json').read_bytes())
    entered=[r for r in trace if 33<=r['progress_m']<=35]
    telemetry=[json.loads(v['data']) for v in rows(folder/'drivetrain_gz.jsonl')]
    a=entered[0]['dynamics']['sim_time_s'];b=entered[-1]['dynamics']['sim_time_s']
    window=[r for r in telemetry if a<=r['sim_time_s']<=b]
    bad=[r for r in window if abs(r['truth_longitudinal_mps'])>1.5
         and min(abs(r[k]*.0325) for k in ('left_speed_rad_s','right_speed_rad_s'))<.5]
    rotating=[abs(r['truth_longitudinal_mps']) for r in window]
    states=Counter(r['autonomy'].get('state') for r in trace)
    stop=report.get('failure_cleanup_stop',{})
    cap=[r for r in trace if r['autonomy'].get('corner_approach')]
    result=dict(progress_m=report['progress_m'],completed_laps=report['completed_laps'],
        peak_body_mps=report['peak_ground_speed_mps'],collision_samples=report['collision_samples'],
        final_state=trace[-1]['autonomy'].get('state'),final_reason=trace[-1]['safety'].get('reason'),
        final_partial=trace[-1]['safety'].get('partial_observation'),failure_stop=stop,
        shutdown_clean=report['shutdown_clean'],source_unchanged=report['source_unchanged'],
        surface_samples=surface['samples'],wall_wheel_samples=surface['wall_wheel_samples'],
        max_off_wheels={k:v['max_off_wheels'] for k,v in surface['interpretations'].items()},
        original_contact_area=dict(progress=[33,35],time=[a,b],drive_samples=len(window),one_wheel_slow_samples=len(bad),
             maximum_speed_mps=max(rotating),minimum_speed_mps=min(rotating)),
        corner_approach_status_samples=len(cap),
        motion_states=dict(states),report_pass=report['passed'],
        input_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in (folder/'report.json',folder/'trace.json',folder/'drivetrain_gz.jsonl',root/'surface.json')})
    # 접촉자료 전체에서 구동 공통시각의 존재를 비교한다. 미수신을 0 N으로 변환하지 않는다.
    times={round(r['sim_time_s']*1e9) for r in window}
    contacts={name:set() for name in ('front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel')}
    with (folder/'wheel_contacts.jsonl').open('rb') as f:
        for line in f:
            row=json.loads(line)
            stamp=row['header']['stamp'];t=int(stamp.get('sec',0))*10**9+int(stamp.get('nsec',0))
            if t not in times:continue
            for c in row.get('contact',[]):
                for field in ('collision1','collision2'):
                    name=c[field]['name'].split('::')
                    if len(name)>=3 and name[0]=='arena_car' and name[1] in contacts:
                        contacts[name[1]].add(t)
    result['original_contact_area']['contact_present_stamps']={k:len(v) for k,v in contacts.items()}
    return result


if __name__=='__main__':
    output={k:review(root) for k,root in [('before',HERE.parent/'solo_contact_diagnosis_v1'),('r19',HERE)]}
    output['scope']='two separate closed-loop runs, same selected geometry/vehicle and contact instrumentation; not repeated statistics or hardware proof'
    (HERE/'comparison.json').write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:{n:v for n,v in r.items() if n in ('progress_m','peak_body_mps','original_contact_area','max_off_wheels','final_reason')} for k,r in output.items() if isinstance(r,dict)},ensure_ascii=False))
