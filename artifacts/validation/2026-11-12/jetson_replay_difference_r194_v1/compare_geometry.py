"""좌표 근접 비교는 원인 진단용이며 제어기의 표본 병합을 바꾸지 않는다."""
import json
from pathlib import Path
import numpy as np

A=Path(__file__).resolve().parent
left=np.load(A/'pc_geometry/queries.npz');right=np.load(A/'jetson_geometry/queries.npz')
assert left.files==right.files
def nearest(source,target):
    buckets={};scale=1e-10
    for q in target:
        key=tuple(np.floor(q/scale).astype('int64'))
        buckets.setdefault(key,[]).append(q)
    distances=[]
    for q in source:
        key=tuple(np.floor(q/scale).astype('int64'))
        options=[p for x in (-1,0,1) for y in (-1,0,1)
                 for p in buckets.get((key[0]+x,key[1]+y),[])]
        if not options:raise AssertionError('difference larger than diagnostic neighbourhood')
        distances.append(float(np.min(np.linalg.norm(np.asarray(options)-q,axis=1))))
    return max(distances)
rows=[]
for name in left.files:
    l,r=left[name],right[name]
    a,b=nearest(l,r),nearest(r,l)
    rows.append(dict(key=name,pc_count=len(l),jetson_count=len(r),
        pc_to_jetson_max_nearest_m=a,jetson_to_pc_max_nearest_m=b,
        hausdorff_distance_m=max(a,b)))
report=dict(seq=401,rows=rows,scope='single_call_geometry_diagnostic_not_general_numeric_equivalence',
    controller_rounding_changed=False)
with (A/'geometry_comparison.json').open('x') as out:json.dump(report,out,indent=2);out.write('\n')
print(json.dumps(report))
