"""실행별 상태·제한 항목을 구간으로 묶어 보여 준다(사후 분석)."""
import csv
import os
import sys

name, a, b = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
rows = list(csv.DictReader(open(os.path.expanduser(f'~/arena_scratch/fc1/{name}.csv'), encoding='utf-8')))
last = None
start = None
acc = []
def flush():
    if last is None:
        return
    o = [x[0] for x in acc]; v = [x[1] for x in acc]
    print(f"{start:6.2f}-{acc[-1][2]:6.2f} n={len(acc):3d} out {min(o):.2f}-{max(o):.2f} body {v[0]:.2f}->{v[-1]:.2f} {last}")
for r in rows:
    t = float(r['t'])
    if t < a or t > b:
        continue
    key = (r['state'], r['bind'][:34], (r['normal_reason'] or '')[:26])
    if key != last:
        flush(); last = key; start = t; acc = []
    acc.append((float(r['out']), float(r['body']), t))
flush()
