import json, numpy as np
rows=[json.loads(l) for l in open('raw/r41_timers_per_call.jsonl')]
tot=np.array([r['total_ms'] for r in rows])
def g(r,k): return r['incl_ms'].get(k,0.)
mr=np.array([g(r,'motion_region') for r in rows]); og=np.array([g(r,'ObservationGrid.__init__') for r in rows])
print('motion_region present', (mr>0).sum(), 'when present mean/p50/max', mr[mr>0].mean(), np.median(mr[mr>0]), mr.max())
p95=np.percentile(tot,95); sel=tot>=p95
print('p95',p95,'n top5%',sel.sum(),'with motion_region', (mr[sel]>0).sum(), 'mean total top', tot[sel].mean(), 'mean mr top', mr[sel].mean(), 'mean og top', og[sel].mean())
print('p50/p95 total minus motion_region', np.percentile(tot-mr,50), np.percentile(tot-mr,95))
print('with grid', (og>0).sum(), 'total p50 with/without grid', np.median(tot[og>0]), np.median(tot[og==0]))
top=np.argsort(tot)[::-1]
for i in top[:6]:
    r=rows[i]; print(r['seq'], round(r['total_ms'],1), {k:round(v,1) for k,v in sorted(r['incl_ms'].items(), key=lambda x:-x[1])[:8]})
