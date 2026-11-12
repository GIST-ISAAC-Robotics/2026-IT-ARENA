"""초기20회전만으로 표본 시계 후보를 맞추고 이후 저장 회전에서 외삽 오차를 확인.

SDK 앵커 자체가 호스트 추정이므로 실제 취득시각/절대 지연 교정이 아니다.
입력의 거리/각도·시각을 덮어쓰지 않고 후보 시각의 모순/잔차만 계산한다.
"""
import hashlib
import json
from pathlib import Path
import statistics

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
PREFIX=20

def stats(values):
    s=sorted(values)
    return dict(n=len(s),min=s[0],median=statistics.median(s),p95=s[int((len(s)-1)*.95)],max=s[-1]) if s else dict(n=0)

def analyze(folder):
    raw=ROOT/folder/'sdk_raw.jsonl';mapped=ROOT/folder/'converted.jsonl'
    source=[json.loads(s) for s in raw.open()];received=[json.loads(s) for s in mapped.open()]
    assert len(source)==len(received)>PREFIX
    tick=0;rows=[]
    for r,m in zip(source,received):
        n=len(r['nodes']);assert len(m['payload']['ranges_m'])==n
        rows.append(dict(tick=tick,n=n,first_ns=r['sdk_estimated_first_us']*1000,
                         end_ns=r['end_monotonic_ns'],receipt_ns=m['receipt_monotonic_ns']))
        tick+=n
    base=rows[0]['first_ns'];prefix=rows[:PREFIX]
    # 최소8회전 간격의 쌍으로 짧은 수신 지터가 rate에 미치는 영향을 줄인 추정.
    slopes=[(b['first_ns']-a['first_ns'])/(b['tick']-a['tick'])
            for i,a in enumerate(prefix) for j,b in enumerate(prefix) if j-i>=8]
    period=statistics.median(slopes)
    intercept=min(min(r['first_ns']-base-r['tick']*period,
                      r['end_ns']-base-(r['tick']+r['n']-1)*period) for r in prefix)
    actual=[];last=None
    for r in rows[PREFIX:]:
        first=base+intercept+r['tick']*period
        end=base+intercept+(r['tick']+r['n']-1)*period
        actual.append(dict(n=r['n'],first_relative_ns=first-base,last_relative_ns=end-base,
            sdk_first_residual_ms=(r['first_ns']-first)/1e6,
            end_after_grab_ms=(end-r['end_ns'])/1e6,end_after_receipt_ms=(end-r['receipt_ns'])/1e6,
            overlap_previous_ms=None if last is None else (last-first)/1e6))
        last=end
    nominal=[(r['first_ns']+(r['n']-1)*200_000-r['receipt_ns'])/1e6 for r in rows[PREFIX:]]
    report=dict(raw=folder+'/sdk_raw.jsonl',raw_sha256=hashlib.sha256(raw.read_bytes()).hexdigest(),
        mapped_sha256=hashlib.sha256(mapped.read_bytes()).hexdigest(),prefix_batches=PREFIX,
        held_out_batches=len(actual),sample_period_ns=period,intercept_relative_ns=intercept,
        sdk_anchor_residual_ms=stats([r['sdk_first_residual_ms'] for r in actual]),
        candidate_after_grab=stats([r['end_after_grab_ms'] for r in actual]),
        candidate_after_receipt=stats([r['end_after_receipt_ms'] for r in actual]),
        nominal_after_receipt=stats(nominal),
        candidate_future_batches=sum(r['end_after_receipt_ms']>0 for r in actual),
        nominal_future_batches=sum(v>0 for v in nominal),
        candidate_overlapping_batches=sum((r['overlap_previous_ms'] or 0)>0 for r in actual),
        time_contract_verified=False,drive_output=False,
        assumptions=['physical revolution continuity and stable source sampling rate are not verified',
                     'SDK anchors are host-decoding estimates, not sensor truth',
                     'no rate/offset fit uses held-out batches; fixed-model timestamps do not react to extra receipt delay'])
    return report,actual

def main():
    reports=[]
    for name in ('r249','r251'):
        report,rows=analyze(f'build/{name}_recovered/c1');reports.append(report)
        (A/(name+'_rows.json')).write_text(json.dumps(rows,indent=2)+'\n')
    (A/'report.json').write_text(json.dumps(reports,indent=2)+'\n');print(json.dumps(reports,indent=2))

if __name__=='__main__':main()
