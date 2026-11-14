import hashlib,json,shutil,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent
S=A.parent/'jetson_traceoff_continuity_r522_v1/bundle'
M=json.loads((S/'manifest.json').read_bytes())
assert all(hashlib.sha256((S/n).read_bytes()).hexdigest()==h for n,h in M['files'].items())
B=A/'bundle';B.mkdir(exist_ok=False)
for name in M['files']:shutil.copyfile(S/name,B/name)
shutil.copyfile(A/'test_budget_candidate.py',B/'test_budget_candidate.py')
p=B/'node_runner.py';s=p.read_text(encoding='utf8')
old='            drained=drain_ready(self._pending_motion_subscriptions)'
new="            drained=drain_ready(self._pending_motion_subscriptions,budget_s=.006 if args.role=='controller' else .002)"
assert s.count(old)==1;s=s.replace(old,new,1)
p.write_bytes(s.encode('utf8'))
M.update(experiment='r523 controller drain budget6ms, guard2ms; r522 trace-OFF exact baseline',
    r522_parent_manifest_sha256=hashlib.sha256((S/'manifest.json').read_bytes()).hexdigest(),
    r522_parent_file_sha256=M['files'].copy(),controller_drain_budget_ms=6.,guard_drain_budget_ms=2.,
    sample_limit=64,budget_evidence_sha256=hashlib.sha256((A/'budget_evidence.json').read_bytes()).hexdigest(),
    test_limit='one trace-OFF 20s trial; no parameter sweep or follow-on trial; current common r518/r521 excluded')
M['files']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in B.iterdir() if p.is_file()}
(B/'manifest.json').write_bytes((json.dumps(M,indent=2,ensure_ascii=False)+'\n').encode('utf8'))
(A/'manifest.json').write_bytes((B/'manifest.json').read_bytes())
for p in B.glob('*.py'):compile(p.read_bytes(),str(p),'exec')
unchanged=[n for n,h in M['r522_parent_file_sha256'].items() if M['files'][n]==h]
assert len(unchanged)==12 and set(M['r522_parent_file_sha256'])-set(unchanged)=={'node_runner.py'}
with zipfile.ZipFile(A/'bundle.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in B.iterdir():z.write(p,p.name)
audit=dict(unchanged_parent_files=unchanged,changed_parent_files=['node_runner.py'],added=['test_budget_candidate.py'],
    zip_sha256=hashlib.sha256((A/'bundle.zip').read_bytes()).hexdigest())
(A/'source_copy_audit.json').write_bytes((json.dumps(audit,indent=2)+'\n').encode('utf8'))
print(json.dumps(audit))
