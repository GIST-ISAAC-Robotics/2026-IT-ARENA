"""Same geometric agreement for both currently observed wall choices; detached."""
from pathlib import Path
import hashlib
import json

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
source=ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py'
old=source.read_bytes();s=old.decode('utf8')
assert not (A/'before_local_follow.py').exists()
(A/'before_local_follow.py').write_bytes(old)
first='eligible = eligible and path_agrees(candidate, carried, cfg.boundary_agreement_m)'
replacement='eligible = eligible and bool(retained_boundary_agreement(candidate, carried, info, cfg))'
assert s.count(first)==1;s=s.replace(first,replacement)
first='ok = sufficient(path, details) and path_agrees(path, reference, cfg.boundary_agreement_m)'
replacement='ok = sufficient(path, details) and bool(retained_boundary_agreement(path, reference, details, cfg))'
assert s.count(first)==2;s=s.replace(first,replacement)
(A/'candidate_local_follow.py').write_bytes(s.encode('utf8'))
template=A.parent/'retained_corner_agreement_probe_v1'
(A/'loader.py').write_bytes((template/'loader.py').read_bytes().replace(b'detached_retained_corner',b'detached_current_corner'))
s=(template/'replay.py').read_text(encoding='utf8').replace('corner_occlusion_connected_r45_v1','retained_corner_agreement_r46_v1')
(A/'replay.py').write_text(s,encoding='utf8',newline='\n')
audit=A.parent/'corner_r46_transition_audit_v1'
row=next(r for r in json.loads((audit/'path_comparison.json').read_bytes()) if abs(r['t']-37.52)<1e-8)
ref=next(r['carried_path'] for r in json.loads((audit/'captures.json').read_bytes()) if abs(r['t']-37.52)<1e-8)
with (A.parent/'retained_corner_agreement_r46_v1/run1/follow_decisions.jsonl').open() as f:
    cfg=json.loads(next(f))['data']['follow_config']
fixture=dict(t=row['t'],reference=ref,current=next(c for c in row['candidates'] if c['side']=='left' and c['kind']=='all'),config=cfg)
(A/'fixture.json').write_text(json.dumps(fixture,indent=2),encoding='utf8')
(A/'design.json').write_text(json.dumps(dict(source_sha256=hashlib.sha256(old).hexdigest(),
    compare_sites=['preferred_current','supported_current','refined_current'],
    thresholds_changed=False,default_options_changed=False,source_modified=False),indent=2),encoding='utf8')
print('Detached three comparison sites; current source unchanged.')
