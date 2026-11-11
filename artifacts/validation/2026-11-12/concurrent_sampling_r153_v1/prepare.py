"""r146 동시 수신에서 거리 없이 실제 각도 배열을 골라 선택형 프로필을 만든다."""
from collections import Counter
import csv
import hashlib
import itertools
import json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'src/arena_vehicle_interface'))
from arena_vehicle_interface.native_lidar_sampling import NativeRevolutionAssembler

P=ROOT/'build/sensor_concurrency_r146/remote/it_arena_sensor_concurrency_r146_20261112_v2/capture'
scan=P/'c1-scans.csv';points=P/'c1-points.csv'
audit=json.loads((A.parent/'sensor_concurrency_r146_v1/audit.json').read_bytes())
hashes={name:hashlib.sha256(p.read_bytes()).hexdigest() for name,p in [('scans',scan),('points',points)]}
assert hashes['scans']==audit['source_sha256']['capture\\c1-scans.csv']
assert hashes['points']==audit['source_sha256']['capture\\c1-points.csv']
measured={int(row['scan']) for row in csv.DictReader(scan.open(newline='')) if row['measured']=='1' and int(row['result'])==0}
all_patterns=[]
with points.open(newline='') as f:
    for sid,group in itertools.groupby(csv.DictReader(f),lambda row:int(row['scan'])):
        if sid not in measured:continue
        rows=list(group)
        assert [int(row['index']) for row in rows]==list(range(len(rows)))
        all_patterns.append((sid,[int(row['angle_q14']) for row in rows]))
assert len(all_patterns)==len(measured)==752
# 각 점수의 첫 사례와 시간축에 고르게 배치한 사례를 합친다.
# 빈도를 그대로 재현하는 통계 모델이 아니라 폭넓은 입력 형식 확인용이다.
selected={};counts_seen=set()
for sid,pattern in all_patterns:
    if len(pattern) not in counts_seen:selected[sid]=pattern;counts_seen.add(len(pattern))
for index in [round(i*(len(all_patterns)-1)/31) for i in range(32)]:
    sid,pattern=all_patterns[index];selected[sid]=pattern
ids=sorted(selected);patterns=[selected[sid] for sid in ids]
NativeRevolutionAssembler(patterns,rate_hz=10.)
record=dict(schema='c1_angular_templates_v1',rotation_rate_hz=10.,time_model='period_divided_by_count',
    hardware_timing_verified=False,
    scope='r146 concurrent capture; first example of each point count plus 32 time-spaced scans. Cyclic shape probe, not frequency/timing/noise/room model.',
    source_scan_ids=ids,source_sha256=hashes,patterns_q14=patterns)
out=A/'profile.json'
with out.open('x') as f:json.dump(record,f,separators=(',',':'));f.write('\n')
result=dict(source_measured_scans=len(all_patterns),source_count_distribution=dict(sorted(Counter(len(p) for _,p in all_patterns).items())),
    selected_scans=len(patterns),selected_count_distribution=dict(sorted(Counter(map(len,patterns)).items())),
    selected_source_scan_ids=ids,profile_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),
    ranges_quality_and_room_geometry_excluded=True,source_unchanged=hashes=={name:hashlib.sha256(p.read_bytes()).hexdigest() for name,p in [('scans',scan),('points',points)]},
    not_adopted_as_default=True,hardware_timing_verified=False)
with (A/'selection.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
