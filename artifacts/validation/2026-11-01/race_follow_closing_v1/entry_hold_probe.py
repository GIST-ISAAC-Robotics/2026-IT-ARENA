"""같은 판단에서 '가장 긴 우회 진입을 남겨두려는 0'만 따로 비교한다. 실제 이동 효과 아님."""
import json
import math
import hashlib
from collections import Counter
from pathlib import Path
from arena_autonomy.local_path import braking_speed

BASE=Path(__file__).resolve().parent


def diagnostic(t,v,q):
    entry=q.get('entry_space',{}); n=q.get('normal',{}); r=q.get('follow_entry_space',{})
    prep=q.get('preparation',{}); forward=q.get('relative_yield',{})
    if not (v==0 and q.get('handoff_state')=='FOLLOW' and entry.get('hold') is True
            and n.get('reason')=='local_follow' and n.get('behavior')!='HOLD'
            and prep.get('applied') is True and r.get('reason')=='observed_entry_distance_reserve'
            and forward.get('cap_requested') is False and not forward.get('final_command_clipped')):
        return v
    # 우회 여유가 남을 때만 접근하고, 기존 감속도·측정 나이로 그 여유 앞에서 멈출 수 있어야 한다.
    dist=r['rear_axle_to_patch_m']-r['reserve_m'];age=t-n['source_first_s']
    if not (math.isfinite(dist) and math.isfinite(age) and 0<=age<.30):return v
    return max(0,min(r['before_mps'],r['cap_mps'],prep['cap_mps'],braking_speed(dist,age)))


def analyze(name):
    folder=BASE/name; rep=json.loads((folder/'report.json').read_text(encoding='utf-8'))
    g=rep['case_window']['green_sim_s'];duration=rep['case_window']['duration_s']
    digest=hashlib.sha256();prev=None;gain=0.;changed=[];zero=Counter()
    with (folder/'follow_decisions.jsonl').open('rb') as stream:
        for line in stream:
            digest.update(line);r=json.loads(line)
            if r['kind']=='reset':prev=None
            if r['kind']!='call':continue
            d=r['data'];v,_,q=d['output'];t=d['input']['reference_s']
            if not 0<=t-g<=duration:continue
            nv=diagnostic(t,v,q)
            if prev is not None and 0<t-prev[0]<=.10:gain+=(t-prev[0])*prev[1]
            prev=t,nv-v
            if q['handoff_state']=='FOLLOW' and v<=.001:
                zero['original_follow_zero']+=1
                if q.get('entry_space',{}).get('hold'):zero['for_future_entry']+=1
            if nv>v+1e-6:
                changed.append(dict(elapsed_s=t-g,proposed_mps=nv,
                    spare_m=q['follow_entry_space']['rear_axle_to_patch_m']-q['follow_entry_space']['reserve_m']))
    assert digest.hexdigest()==rep['follow_decisions']['sha256']
    return dict(case=name,input_sha256=digest.hexdigest(),zero=dict(zero),changed=changed,
                commanded_increment_integral_m=gain)


if __name__=='__main__':
    with (BASE/'entry_hold_probe.json').open('x',encoding='utf-8') as f:
        json.dump(dict(scope='현재 센서 판단만 사용한 정지 명령의 대조. 실제 우회 공간 보존·속도 이득의 증거는 아님.',
                       cases=[analyze(k) for k in ('slow','stop')]),f,ensure_ascii=False,indent=2)
