"""1랩 중 계획기가 1 m/s 이하를 명령한 시간과 당시 제약을 저장 판단으로 구분한다."""
from collections import defaultdict
import argparse
import json
from pathlib import Path


def category(v,d):
    if v<.001:return 'stop_request'
    if v>1.001:return 'above_1'
    if d.get('reason')!='local_follow':return 'other_path:'+str(d.get('reason'))
    if d.get('behavior')=='FOLLOW':return 'occupied_arc_brake_or_headway'
    if not d.get('open_route_speed_enabled'):return 'standard_ceiling_or_curve'
    if abs(v-d.get('observed_length_speed_cap_mps',-2.))<1e-6:return 'approach_length_blend'
    return 'observed_path_braking_or_curvature'


def measure(folder):
    r=json.loads((folder/'report.json').read_text(encoding='utf-8'))
    target=r['lap_length_m'];start=end=None;before=None
    with (folder/'trajectory.jsonl').open(encoding='utf-8') as f:
        for row in map(json.loads,f):
            if row['light']!='green':continue
            if start is None:start=row['time']
            if before and before['progress_m']<target<=row['progress_m']:
                part=(target-before['progress_m'])/(row['progress_m']-before['progress_m'])
                end=before['time']+part*(row['time']-before['time']);break
            before=row
    if end is None:raise ValueError('완주하지 않은 궤적으로 랩 시간을 집계할 수 없음')
    times=defaultdict(float);prev=None
    with (folder/'follow_decisions.jsonl').open(encoding='utf-8') as f:
        for event in map(json.loads,f):
            if event['kind'] not in ('call','reset'):continue
            if event['kind']=='reset':prev=None;continue
            t=event['data']['input']['reference_s']
            v,_,d=event['data']['output']
            if prev and 0<t-prev[0]<=.10:
                dt=max(0.,min(t,end)-max(prev[0],start))
                if dt:times[prev[1]]+=dt
            prev=(t,category(v,d))
            if t>end:break
    total=end-start;miss=total-sum(times.values())
    if miss < -1e-6:raise ValueError('판단 구간 중복')
    return dict(lap_s=total,uncovered_s=max(0.,miss),requested_time_s=dict(sorted(times.items(),
                key=lambda pair:-pair[1])),scope='정답 차체 속도가 아닌 /drive 이전 계획 명령. 여러 제약이 겹친 경우 기록된 우선 분류이며, 제약을 없앴을 때 회수될 시간은 아니다.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('result',type=Path)
    a=p.parse_args();result=measure(a.run)
    with a.result.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps(result,ensure_ascii=False))
