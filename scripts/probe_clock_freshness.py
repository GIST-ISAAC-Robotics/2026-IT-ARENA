#!/usr/bin/env python3
"""시각 구간 신선도 12개 합성 진단. 실제 통신/센서 측정은 아니다."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

from audit_clock_freshness import ROOT, SCHEMA, SOURCES


def cases():
    rows=[dict(kind='header',schema='arena_clock_exchange_audit_v1',origin='synthetic',
        host_clock='host_monotonic_ns',device_clock='mcu_monotonic_us',assumptions=dict(
            max_rate_error_ppm=0,host_timestamp_error_ns=0,mcu_timestamp_error_us=0,
            max_window_us=5_000_000,max_extrapolation_us=100_000)),
        dict(kind='exchange',boot='boot-A',sequence=0,host_send_ns=8_999_000_000,
            mcu_receive_us=1_000_000,mcu_send_us=1_000_100,host_receive_ns=9_009_100_000),
        dict(kind='query',boot='boot-A',capture_us=1_000_050)]
    data=('\n'.join(json.dumps(r) for r in rows)+'\n').encode()
    query=dict(mapped_query_index=0,boot='boot-A',host_epoch='host-A',
        now_lower_ns=9_025_000_000,now_upper_ns=9_025_000_000,
        max_age_ns=30_000_000,limit_inclusive=False,
        policy_note='합성 30 ms 예시. 운영 timeout 변경/실물 허용값 확정 아님')
    for name,now,inclusive,expected in [
        ('fresh',9_025_000_000,False,'fresh_under_assumptions'),
        ('midpoint_would_accept',9_031_000_000,False,'uncertain_age'),
        ('stale',9_050_000_000,False,'stale_under_assumptions'),
        ('future',8_980_000_000,False,'future_under_assumptions'),
        ('future_overlap',9_005_000_000,False,'uncertain_age'),
        ('exclusive_endpoint',9_029_050_000,False,'uncertain_age'),
        ('inclusive_endpoint',9_029_050_000,True,'fresh_under_assumptions'),
        ('stale_endpoint',9_039_050_000,False,'stale_under_assumptions'),
        ('uncertain_now',9_025_000_000,False,'uncertain_age'),
        ('host_reset',9_025_000_000,False,None),
        ('mcu_reset',9_025_000_000,False,None),
        ('other_capture',9_025_000_000,False,None)]:
        current=copy.deepcopy(query)
        current.update(now_lower_ns=now,now_upper_ns=now,limit_inclusive=inclusive)
        if name=='uncertain_now': current['now_upper_ns']+=10_000_000
        if name=='host_reset': current['host_epoch']='host-B'
        if name=='mcu_reset': current['boot']='boot-B'
        spec=dict(schema=SCHEMA,origin='synthetic',exchange_sha256=hashlib.sha256(data).hexdigest(),
                  host_epoch='host-A',clock='host_monotonic_ns',queries=[current])
        if name=='other_capture': spec['exchange_sha256']='0'*64
        yield name,data,spec,expected


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    sources=(*SOURCES,'scripts/probe_clock_freshness.py','tests/test_clock_freshness.py')
    hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as bundle:
        for path in sources: bundle.write(ROOT/path,path)
    results=[]
    for name,data,specification,expected in cases():
        capture,spec,result=[args.output/(name+suffix) for suffix in ('.exchange.jsonl','.review.json','.report.json')]
        capture.write_bytes(data)
        spec.write_text(json.dumps(specification,ensure_ascii=False,indent=2),encoding='utf-8')
        command=[sys.executable,str(ROOT/'scripts/audit_clock_freshness.py'),str(capture),str(spec),'--output',str(result)]
        completed=subprocess.run(command,capture_output=True,text=True,timeout=30)
        (args.output/(name+'.log')).write_text(completed.stdout+completed.stderr,encoding='utf-8')
        actual=json.loads(result.read_bytes()) if result.exists() else {}
        status=actual.get('queries',[{}])[0].get('status')
        passed=(completed.returncode==(0 if expected is not None else 2)
                and actual.get('review_valid')==(expected is not None) and status==expected
                and actual.get('ready_to_drive') is False
                and actual.get('input_unchanged') is True and actual.get('source_unchanged') is True)
        results.append(dict(name=name,passed=passed,expected_status=expected,actual_status=status,
                            review_valid=actual.get('review_valid'),returncode=completed.returncode))
    unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    report=dict(passed=all(r['passed'] for r in results) and unchanged,cases=results,source_sha256=hashes,
                source_unchanged=unchanged,scope='synthetic_interval_review_not_hardware_or_ros_mapping',
                ready_to_drive=False)
    (args.output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report))
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
