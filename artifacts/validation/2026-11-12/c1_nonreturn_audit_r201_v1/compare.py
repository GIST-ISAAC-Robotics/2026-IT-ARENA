"""원시 0거리만 제조사 무반환 표기로 분리한 저장 대조. 운영 코드/시각은 불변."""
from collections import Counter
import copy
import hashlib
import json
import math
from pathlib import Path

from arena_autonomy.local_scene import scene_from_measurements
from arena_autonomy.native_lidar_input import decode_measurements
from arena_vehicle_interface.c1_native_preview import C1NativePreview

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
D=ROOT/'build/r166b_recovered/artifacts/validation/2026-11-12/c1_ros_preview_r166_v1/capture'
counter=Counter();statuses=Counter();converter=None
raws=[json.loads(line) for line in (D/'sdk_raw.jsonl').open()]
converted=[json.loads(line) for line in (D/'converted.jsonl').open()]
assert len(raws)==len(converted)==204
for raw, stored in zip(raws, converted):
    old=stored['payload']
    if isinstance(old,str):old=json.loads(old)
    if converter is None:converter=C1NativePreview(source_epoch=old['source_epoch'])
    regenerated,_=converter.convert(raw, receipt_monotonic_ns=stored['receipt_monotonic_ns'],
                                    receipt_ros_ns=stored['receipt_ros_ns'])
    assert regenerated==old
    new=copy.deepcopy(old)
    for index,(_,distance,quality,flag) in enumerate(raw['nodes']):
        if distance==0:
            assert old['ranges_m'][index] is None
            new['ranges_m'][index]='+inf';counter['changed_zero_distance_points']+=1
        elif quality==0:
            assert new['ranges_m'][index] is None
            counter['still_invalid_positive_distance_points']+=1
        else:
            assert new['ranges_m'][index]==old['ranges_m'][index]
            counter['unchanged_positive_quality_points']+=1
    scenes=[]
    for payload in (old,new):
        scan=decode_measurements(json.dumps(payload,allow_nan=False),expected_time_basis='unverified_estimate')
        scenes.append(scene_from_measurements(scan.ranges,angles_rad=scan.angles_rad,
            range_min=scan.range_min,range_max=scan.range_max,acquired_at_s=scan.acquired_at_s,
            t_ref_s=max(scan.acquired_at_s),preserve_sampling=True))
    assert scenes[0]==scenes[1]
    statuses.update(ray.status for ray in scenes[1].rays)
    assert new['time_basis']=='unverified_estimate'
    counter['same_scene_batches']+=1
summary=dict(counts=dict(counter),ray_status_counts=dict(statuses),
    input_sha256={name:hashlib.sha256((D/name).read_bytes()).hexdigest() for name in ('sdk_raw.jsonl','converted.jsonl')},
    scope='saved raw-backed proposal only; exact same scene rays/hits/unknown classifications; no driving or timestamp approval')
with (A/'comparison.json').open('x') as f:json.dump(summary,f,indent=2);f.write('\n')
print(json.dumps(summary))
