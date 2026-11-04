"""Offline, read-only analysis of completed pilot receipts, no controller input."""
import json,sys,collections
from pathlib import Path
P=Path(__file__).resolve().parent
R=P.parent/'lidar_mount_pilot_v3'
def rows(p):
    with p.open(encoding='utf8') as f:
        for l in f:
            if l.strip(): yield json.loads(l)
def one(n):
    e=json.loads((R/(n+'_eval.json')).read_text(encoding='utf8'))
    assert (R/(n+'.exit')).is_file()
    t=[r for r in rows(R/n/'trajectory.jsonl') if r.get('light')=='green']
    if not t: return dict(evaluation=e.get('error'),green_samples=0)
    start=t[0]['time']
    lasts=start+e['lap_s'] if e.get('lap_s') else t[-1]['time']
    c=collections.Counter()
    for r in rows(R/n/'follow_decisions.jsonl'):
        if r['kind']!='call' or not start<=r['data']['input']['reference_s']<=lasts: continue
        v,_,q=r['data']['output']; c['calls']+=1
        if 'open_route_speed_enabled' in q:
            c['cruise_gate_evaluated']+=1
            c['cruise_gate_enabled']+=int(q['open_route_speed_enabled'])
        po=q.get('partial_observation') or {}
        if (po.get('known_structure') or {}).get('speed_cap_mps') is not None: c['explicit_structure_caps']+=1
    first={}
    for m in [16,25,40]:
        hit=next((r for r in t if r['time']<=lasts and r['progress_m']>=m),None)
        first[m]=round(hit['time']-start,3) if hit else None
    return dict(counters=c,first_green_to_progress_s=first,completed_laps=e['completed_laps'],lap_s=e['lap_s'],peak=e['peak_ground_speed_mps'],
                collision_samples=e['collision_samples'],stable_stop=e['stop_stable'])
if __name__=='__main__':
    (P/'drive_diagnostics.json').write_text(json.dumps({n:one(n) for n in sys.argv[1:]},indent=2)+'\n')
