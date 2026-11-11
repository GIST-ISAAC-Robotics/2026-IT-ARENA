"""Capture the first r84 final latch's memory association, without changing decisions."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/p) for p in ('src/arena_autonomy','src/arena_vehicle_interface','scripts')]
from arena_autonomy import follow_trace as ft
from arena_autonomy.known_structure import StructureAwareMixin

A=root/'artifacts/validation/2026-11-11/guard_masked_speed_r84_v1'
trace=next((A/'two_actor_run1/run1').rglob('follow_decisions.jsonl'))
output=root/'build/r86_structure_capture.json'
assert not output.exists()
hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (root/'src/arena_autonomy/arena_autonomy').glob('*.py')}
trace_hash=hashlib.sha256(trace.read_bytes()).hexdigest()
original=StructureAwareMixin._refine_threat
records=[]
def instrument(self, threatening, carried, *args):
    result=original(self,threatening,carried,*args)
    previous, _, _, _, now=args[:5]
    if 60.35<now<60.37:
        scan, _, rows, _, history, _, source=self.structure_active_call[:7]
        shadows=self._shadow(scan,rows,history,now,source)
        hidden,reference=self.structure_hidden
        selected=np.flatnonzero(np.any(result,axis=0))
        points=np.asarray(carried)[selected]
        nearest=np.min(np.linalg.norm(points[:,None,:]-hidden[None,:,:],axis=2),axis=1) if len(hidden) else np.full(len(points),np.inf)
        records.append(dict(reference_s=now,previous_reference_s=previous[1],previous_source_s=previous[2],
            structure_reference_s=reference,structure_last_source=self.structure_last_source,
            structure_previous_reference=self.structure_previous[1],
            retained=[dict(index=int(i),xy=p.tolist(),inside=self._inside([p],[s for s in shadows if s['explained']]).tolist(),
                           memory_distance_m=float(d)) for i,p,d in zip(selected,points,nearest)],
            hidden=hidden.tolist()))
    return result
StructureAwareMixin._refine_threat=instrument
call=ft.replay_call
count=0
class Finished(Exception):pass
def wrapped(*args,**kwargs):
    global count
    result=call(*args,**kwargs);count+=1
    if records:
        raise Finished()
    return result
ft.replay_call=wrapped
try:
    with trace.open(encoding='utf8') as stream:
        ft.replay_events(json.loads(line) for line in stream)
except Finished:
    pass
finally:
    StructureAwareMixin._refine_threat=original;ft.replay_call=call
assert records
unchanged=all(hashlib.sha256((root/n).read_bytes()).hexdigest()==v for n,v in hashes.items())
assert unchanged and hashlib.sha256(trace.read_bytes()).hexdigest()==trace_hash
output.write_text(json.dumps(dict(scope='state capture at saved latch; no driving or policy change',
    calls=count,source_unchanged=unchanged,input_sha256=trace_hash,records=records),indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(calls=count,records=len(records),output=str(output))))
