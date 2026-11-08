import json, numpy as np
rows=[json.loads(l) for l in open('raw/r41_timers_per_call.jsonl')]
tot=np.array([r['total_ms'] for r in rows])
side=[r['side'] for r in rows]
c3=np.array([s['rear_masked_loop_ms']-s['parsed_prototype_loop_ms'] for s in side])
og=np.array([r['incl_ms'].get('ObservationGrid.__init__',0.) for r in rows])
oc=np.array([r['by_parent_ms'].get('ObservationGrid.__init__ > compact_return_mask',0.) for r in rows])
mr=np.array([r['incl_ms'].get('motion_region',0.) for r in rows])
out={}
def d(x): return dict(mean=float(x.mean()),p50=float(np.percentile(x,50)),p95=float(np.percentile(x,95)),over_20ms=int((x>20).sum()))
out['timer_pass_total']=d(tot)
out['minus_C3_blind_spec']=d(tot-c3)
out['minus_C2_compact_reuse_only_compact_part']=d(tot-oc)
out['minus_C2_C3']=d(tot-c3-oc)
for frac in (0.25,0.5):
    out[f'minus_C2_C3_and_{int(frac*100)}pct_of_motion_region']=d(tot-c3-oc-frac*mr)
out['C3_saving_ms']=d(c3); out['C2_compact_ms_when_grid']=d(oc[og>0])
out['motion_region_ms_when_present']=d(mr[mr>0])
print(json.dumps(out,indent=1))
json.dump(out,open('counterfactual_estimates.json','x'),indent=1)
