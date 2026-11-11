#!/usr/bin/env python3
"""SDK 기록에서 거리·실내 좌표 없이 각도/점수만 선택형 시뮬 템플릿으로 추출한다."""
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path


def prepare(scans,points):
    measured={int(r['scan']) for r in csv.DictReader(scans.open(newline=''))
              if r['measured']=='1' and int(r['result'])==0}
    patterns=[];ids=[]
    with points.open(newline='') as f:
        for sid,group in itertools.groupby(csv.DictReader(f),lambda r:int(r['scan'])):
            if sid not in measured: continue
            rows=list(group)
            if [int(r['index']) for r in rows]!=list(range(len(rows))):
                raise ValueError('missing source indices')
            if len(patterns)<32:
                patterns.append([int(r['angle_q14']) for r in rows]);ids.append(sid)
    return dict(schema='c1_angular_templates_v1',rotation_rate_hz=10.,
        time_model='period_divided_by_count',hardware_timing_verified=False,
        scope='first 32 measured 10Hz angular arrays; ranges/quality/room geometry excluded; repeated cyclically',
        source_scan_ids=ids,source_sha256={k:hashlib.sha256(v.read_bytes()).hexdigest() for k,v in [('scans',scans),('points',points)]},
        patterns_q14=patterns)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('scans','points','output'): p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('output already exists')
    result=prepare(a.scans,a.points);a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,separators=(',',':'))+'\n',encoding='utf-8')
    print(json.dumps(dict(patterns=len(result['patterns_q14']),counts=sorted(set(map(len,result['patterns_q14']))))))
