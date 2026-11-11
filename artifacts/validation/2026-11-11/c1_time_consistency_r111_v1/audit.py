"""실측 배치 시각에 명목 200us를 붙였을 때의 일관성. 취득 시각 교정은 아니다."""
import csv
import hashlib
import json
import statistics
from pathlib import Path

A=Path(__file__).resolve().parent
ROOT=A.parents[3]


def summary(values):
    q=sorted(values)
    return dict(count=len(q),minimum=q[0],median=statistics.median(q),
        p95=q[min(len(q)-1,int(.95*len(q)))],maximum=q[-1])


def audit(path):
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    with path.open(newline='') as stream:
        rows=[r for r in csv.DictReader(stream) if r['measured']=='1'
              and int(r['result'])==0 and int(r['count'])>0]
    if not rows:raise ValueError('no measured successful batch')
    tail=[];seams=[];interval_steps=[]
    for r in rows:
        first=int(r['sdk_estimated_first_us'])/1e6
        last=first+(int(r['count'])-1)*.0002
        tail.append(1000*(int(r['end_monotonic_ns'])/1e9-last))
    for a,b in zip(rows,rows[1:]):
        if int(b['scan'])!=int(a['scan'])+1:continue
        dt=(int(b['sdk_estimated_first_us'])-int(a['sdk_estimated_first_us']))/1e6
        if dt<=0:raise ValueError('batch anchor is not increasing')
        seams.append(1000*(dt-(int(a['count'])-1)*.0002))
        interval_steps.append(dt/int(a['count'])*1e6)
    assert hashlib.sha256(path.read_bytes()).hexdigest()==digest
    return dict(source=path.relative_to(ROOT).as_posix(),sha256=digest,scans=len(rows),
        nominal_sample_us=200.,nominal_tail_to_next_first_ms=summary(seams),
        nonincreasing_interbatch_pairs=sum(x<=0 for x in seams),
        nominal_tail_to_grab_end_ms=summary(tail),nominal_tail_after_grab_batches=sum(x<0 for x in tail),
        anchor_interval_divided_by_count_us=summary(interval_steps),
        calibration_applied=False,physical_acquisition_timing_verified=False)


def main():
    base=ROOT/'build/c1_hardware_2026_11_10'
    paths=[base/'remote/raw_startup_checked-scans.csv']
    paths.extend(base/'speed_remote/speed_retry'/name for name in
                 ('standard8-scans.csv','delayed12-scans.csv','final10-scans.csv'))
    result=dict(scope='saved SDK batch/host clock consistency, not sensor timestamp calibration',
        results=[audit(p) for p in paths])
    with (A/'report.json').open('x',encoding='utf8') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result))


if __name__=='__main__':main()
