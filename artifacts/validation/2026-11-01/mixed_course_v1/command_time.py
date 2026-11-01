"""155초 동안 실제 명령을 제한한 상태의 시간을 집계한다. 원시 로그의 구속조건 기록에 한정."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path

BASE=Path(__file__).resolve().parent


def group(v,d):
    state=d.get('handoff_state')
    if state!='FOLLOW':return str(state)
    normal=d.get('normal',{});space=d.get('follow_entry_space',{});prep=d.get('preparation',{})
    if v<=.001:
        if d.get('entry_space',{}).get('hold'):return 'FOLLOW/zero_future_entry'
        if space.get('applied'):return 'FOLLOW/zero_'+space.get('reason','space')
        return 'FOLLOW/zero_'+normal.get('reason','normal')
    if space.get('applied'):return 'FOLLOW/'+space.get('reason','space')
    if prep.get('applied'):return 'FOLLOW/future_bypass_preparation'
    return 'FOLLOW/current_path'


def analyze(folder):
    r=json.loads((folder/'report.json').read_text())
    start=r['case_window']['green_sim_s'];end=r['case_window']['end_sim_s']
    times=defaultdict(float);values=defaultdict(list);prev=None;sha=hashlib.sha256()
    with (folder/'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            sha.update(line);e=json.loads(line)
            if e['kind']=='reset':prev=None
            if e['kind']!='call':continue
            d=e['data'];t=d['input']['reference_s'];v,_,status=d['output'];name=group(v,status)
            if prev and 0<t-prev[0]<=.10:
                dt=max(0.,min(t,end)-max(start,prev[0]))
                times[prev[1]]+=dt
            if start<=t<=end:values[name].append(v)
            prev=t,name
    assert sha.hexdigest()==r['follow_decisions']['sha256']
    return dict(total_window_s=end-start,covered_s=sum(times.values()),uncovered_s=end-start-sum(times.values()),
        categories={k:dict(seconds=dt,samples=len(values[k]),minimum_command=min(values[k],default=None),
                           maximum_command=max(values[k],default=None)) for k,dt in sorted(times.items(),key=lambda kv:-kv[1])},
        trace_sha256=sha.hexdigest(),scope='최종 50 Hz 발행 가속·보호 출력 전 계획 판단. 우선 표시 조건의 시간이며 이를 제거했을 때 랩 이득은 아니다.')


if __name__=='__main__':
    with (BASE/'command_time.json').open('x',encoding='utf-8') as f:
        json.dump(analyze(BASE/'run1'),f,ensure_ascii=False,indent=2)
