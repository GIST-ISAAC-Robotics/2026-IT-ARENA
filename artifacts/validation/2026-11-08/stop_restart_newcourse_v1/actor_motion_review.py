"""원 평가를 보존한 실제 배우 이동 재검토. 짧은 차분 대신1초 위치 변화, 같은0.45m/s 기준."""
from bisect import bisect_left
import hashlib,json,math
from pathlib import Path

HERE=Path(__file__).resolve().parent
RUN=HERE/'moving_stop_restart/run1'
read=lambda p:json.loads(p.read_bytes())
report=read(RUN/'report.json'); original=read(RUN.parent/'assessment_run1.json')
rows=[json.loads(l) for l in (RUN/'actor_poses.jsonl').read_bytes().splitlines()]
times=[r[0] for r in rows];green=report['evaluation']['t_green']

def position(t):
    j=bisect_left(times,t)
    assert 0<j<len(rows)
    a,b=rows[j-1:j+1]
    assert 0<b[0]-a[0]<=.12
    u=(t-a[0])/(b[0]-a[0])
    return [a[k]+u*(b[k]-a[k]) for k in (1,2)]

windows=[]
for start in [.5,1.,1.5]+[8.5+.5*k for k in range(17)]:
    lo,hi=green+start,green+start+1.
    observed=[t for t in times if lo-.12<=t<=hi+.12]
    assert observed and max(b-a for a,b in zip(observed,observed[1:]))<=.12
    windows.append(dict(from_green_s=start,to_green_s=start+1.,
                        measured_speed_mps=math.dist(position(lo),position(hi))))
advanced=all(w['measured_speed_mps']>.45 for w in windows)
raw=[]
for a,b in zip(rows,rows[1:]):
    t=a[0]-green
    if (.5<=t<=2.5) or (8.5<=t<=17.5):
        assert b[0]>a[0]
        raw.append(math.dist(a[1:3],b[1:3])/(b[0]-a[0]))
conditions=dict(original['scene_conditions'],actor_advanced=advanced)
gates={k:bool(v) for k,v in original['gates'].items()}
established=all(v for v in conditions.values() if isinstance(v,bool))
result=dict(scope='사후 평가 방법 교정. 원 assessment/run1_goal은 불변; 실제 주행 재실행 아님.',
    reason='50Hz 각본 위치 갱신/100Hz 자세 표본의 반복 위치를 순간 미분한 최소값은 이동 속도 성립 검사가 될 수 없음',
    original_assessment_sha256=hashlib.sha256((RUN.parent/'assessment_run1.json').read_bytes()).hexdigest(),
    actor_poses_sha256=hashlib.sha256((RUN/'actor_poses.jsonl').read_bytes()).hexdigest(),
    instantaneous_raw_min_mps=min(raw),instantaneous_zero_intervals=sum(v==0 for v in raw),
    instantaneous_intervals=len(raw),windows=windows,scene_conditions=conditions,
    established=established,gates=gates,passed=established and all(gates.values()),
    integration=original['integration'],metrics=original['metrics'])
result['supplementary_all_true']=result['passed'] and all(result['integration'].values())
(HERE/'actor_motion_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:result[k] for k in ('instantaneous_raw_min_mps','instantaneous_zero_intervals','instantaneous_intervals','established','passed')},ensure_ascii=True))
print('window speed range',min(w['measured_speed_mps'] for w in windows),max(w['measured_speed_mps'] for w in windows))
