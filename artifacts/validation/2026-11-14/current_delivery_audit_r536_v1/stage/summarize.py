"""단1회 저장 결과의 긴 양수 공백·0 목표 시간과 전달 경계만 집계한다."""
from collections import Counter
import gzip
import json
from pathlib import Path
import sys

START, END = 7.62, 27.52


def quantiles(values):
    values = sorted(values)
    if not values:
        return dict(n=0)
    def at(q):
        p = (len(values)-1)*q
        i = int(p)
        return values[i]+(values[min(i+1,len(values)-1)]-values[i])*(p-i)
    return dict(n=len(values),median=at(.5),p95=at(.95),maximum=values[-1])


def zero_intervals(rows):
    positive = next((r for r in rows if r['speed']>0), None)
    if positive is None:
        return dict(first_positive=None,intervals=[])
    active, intervals = [], []
    for row in rows:
        if row['received_s'] < positive['received_s']:
            continue
        if row['speed']==0:
            active.append(row)
        elif active:
            intervals.append(dict(start_s=active[0]['received_s'],end_s=row['received_s'],
                                  count=len(active),right_censored=False))
            active=[]
    if active:
        intervals.append(dict(start_s=active[0]['received_s'],end_s=END,count=len(active),right_censored=True))
    total=sum(r['end_s']-r['start_s'] for r in intervals)
    return dict(first_positive=positive['received_s'],intervals=intervals,total_zero_s=total,
                active_zero_fraction=total/(END-positive['received_s']),
                closed_interval_ms=quantiles((r['end_s']-r['start_s'])*1000 for r in intervals if not r['right_censored']))


def summarize(root):
    def gz(name):
        with gzip.open(root/(name+'.jsonl.gz'),'rt',encoding='utf8') as stream:
            return [json.loads(line) for line in stream]
    def scalar(role):
        return [json.loads(line) for line in (root/('scalar_'+role+'.jsonl')).read_text(encoding='utf8').splitlines()]
    commands=gz('commands')
    guard=[r['value'] for r in gz('guard_status') if START<=r['value']['reference_s']<END]
    control=scalar('controller'); safety=scalar('guard')
    result=dict(scope='saved simulation replay; no physical stopping-time proof',active_interval_s=[START,END],commands={})
    for topic in ('/drive','/drive/safe'):
        selected=[r for r in commands if r['topic']==topic and START<=r['received_s']<END]
        positive=[r for r in selected if r['speed']>0]
        result['commands'][topic]=dict(total=len(selected),zero=sum(r['speed']==0 for r in selected),positive=len(positive),
            positive_gap_ms=quantiles((b['received_s']-a['received_s'])*1000 for a,b in zip(positive,positive[1:])),
            positive_header_age_ms=quantiles((r['received_s']-r['source_s'])*1000 for r in positive),
            zero_target=zero_intervals(selected))
    result['safe_header_age_is_not_original_command_age']=True
    result['guard_reasons']=dict(Counter(r['reason'] for r in guard))
    result['command_stale']=sum(r.get('command_stale') is True for r in guard)
    evaluated={round(r['reference_s']*1e9):r for r in safety if r['event']=='control_end' and r.get('reference_s') is not None}
    ages=[]
    for row in guard:
        if row.get('command_stale') is not True:
            continue
        receipt=evaluated.get(round(row['reference_s']*1e9))
        if receipt and receipt.get('command_source_s') is not None:
            ages.append((row['reference_s']-receipt['command_source_s'])*1000)
    result['stale_original_command_age_ms']=quantiles(ages)
    result['controller_motion_stop']=sum(r.get('state')=='MOTION_STOP' for r in control
        if r['event']=='control_end' and START<=r.get('reference_s',-1)<END)
    ready={r['token']:r for r in control if r['event']=='complete' and r.get('stage')=='current'}
    permits={r['token']:r for r in control if r['event']=='permit' and r.get('kind')=='candidate'}
    publications=[r for r in control if r['event']=='controller_publish_begin' and r.get('token') in ready
                  and START<=r['ros_s']<END]
    result['current_delivery']=dict(
        published_tokens=len(publications),
        collection_to_publish_ms=quantiles((r['monotonic_ns']/1e9-ready[r['token']]['received_wall_s'])*1000 for r in publications),
        permit_to_publish_ms=quantiles((r['monotonic_ns']-permits[r['token']]['monotonic_ns'])/1e6
            for r in publications if r['token'] in permits),
        permit_reasons=dict(Counter(r.get('reason') for r in control if r['event']=='permit' and START<=r['ros_s']<END)))
    received={r['original_command_source_ns']:r for r in safety if r['event']=='guard_receive_begin'}
    accepted={r['original_command_source_ns'] for r in safety if r['event']=='guard_receive_end'
              and r['stored_command_source_ns']==r['original_command_source_ns']}
    delivered=[r for r in publications if r['header_source_ns'] in received and r['header_source_ns'] in accepted
               and received[r['header_source_ns']]['monotonic_ns']>=r['monotonic_ns']]
    result['current_delivery']['publication_to_guard_receive_ms']=quantiles(
        (received[r['header_source_ns']]['monotonic_ns']-r['monotonic_ns'])/1e6 for r in delivered)
    result['current_delivery']['accepted_guard_matches']=len(delivered)
    result['completion']=json.loads((root/'execution.json').read_bytes())
    result['scalar_integrity']={role:json.loads((root/(role+'_scalar_summary.json')).read_bytes()) for role in ('controller','guard')}
    result['trace_bytes']=(root/'controller_trace.jsonl').stat().st_size
    (root/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:result[k] for k in ('controller_motion_stop','command_stale','current_delivery','scalar_integrity')},ensure_ascii=False))


if __name__=='__main__':
    summarize(Path(sys.argv[1]))
