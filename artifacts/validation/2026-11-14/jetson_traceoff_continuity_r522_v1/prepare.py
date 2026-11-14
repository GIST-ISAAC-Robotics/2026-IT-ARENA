"""r508 최종 동결 stage를 복사하고 상세 trace sink만 바꾼다."""
import hashlib,json,shutil,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent
S=A.parent/'jetson_input_continuity_r508_v1/candidate_shm16_drain_proof'
M=json.loads((S/'manifest.json').read_bytes())
assert all(hashlib.sha256((S/n).read_bytes()).hexdigest()==h for n,h in M['files'].items())
B=A/'bundle';B.mkdir(exist_ok=False)
for name in M['files']:shutil.copyfile(S/name,B/name)
shutil.copyfile(A/'trace_off_sink.py',B/'trace_off_sink.py')
p=B/'node_runner.py';s=p.read_text(encoding='utf8')
old='            class ObservedG1(ControlReceipt,G1G2Node):pass'
new='''            class ObservedG1(ControlReceipt,G1G2Node):
                def create_follow_trace(self,trace_path):
                    # 경로/모드/typed 구성 검사를 원 생성자로 모두 수행한다.
                    trace=super().create_follow_trace(trace_path)
                    trace.close()
                    self.trace_creation_receipt=dict(path=str(trace_path),
                        startup_only_bytes=Path(trace_path).stat().st_size,
                        constructor_validation_preserved=True)
                    from trace_off_sink import TraceOff
                    return TraceOff()
'''
assert s.count(old)==1;s=s.replace(old,new,1)
s=s.replace("            summary['delivery_counts'] = dict(owner.decision_delivery.counts)",
    "            summary['trace_off'] = dict(owner.follow_trace.snapshot(),creation=owner.trace_creation_receipt)\n"
    "            summary['delivery_counts'] = dict(owner.decision_delivery.counts)",1)
s=s.replace("        write(out / (args.role + '_ready.json'), summary)",
    "        if args.role=='controller':\n"
    "            summary['trace_off']=dict(owner.follow_trace.snapshot(),creation=owner.trace_creation_receipt)\n"
    "        write(out / (args.role + '_ready.json'), summary)",1)
p.write_bytes(s.encode('utf8'))
M.update(experiment='r522 trace payload OFF only; r508 final SHM16+drain+proof retained',
    r508_parent_manifest_sha256=hashlib.sha256((S/'manifest.json').read_bytes()).hexdigest(),
    r508_parent_file_sha256=M['files'].copy(),
    excluded_followups=['r518 fit changes','r521 wall reacquisition','current common-source integration'],
    trace_off_mechanism='empty g1_trace_path forbidden by fixed r500 gates; original validated constructor writes header/footer only then test-only no-payload sink',
    test_limit='one 20s requested selection, source-covered active window 7.62<=t<27.52; no repeat')
M['files']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in B.iterdir() if p.is_file()}
(B/'manifest.json').write_bytes((json.dumps(M,indent=2,ensure_ascii=False)+'\n').encode('utf8'))
(A/'manifest.json').write_bytes((B/'manifest.json').read_bytes())
for p in B.glob('*.py'):compile(p.read_bytes(),str(p),'exec')
unchanged=[n for n,h in M['r508_parent_file_sha256'].items() if M['files'][n]==h]
assert len(unchanged)==11 and set(M['r508_parent_file_sha256'])-set(unchanged)=={'node_runner.py'}
with zipfile.ZipFile(A/'bundle.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in B.iterdir():z.write(p,p.name)
audit=dict(unchanged_parent_files=unchanged,changed_parent_files=['node_runner.py'],added=['trace_off_sink.py'],
    zip_sha256=hashlib.sha256((A/'bundle.zip').read_bytes()).hexdigest())
(A/'source_copy_audit.json').write_bytes((json.dumps(audit,indent=2)+'\n').encode('utf8'))
print(json.dumps(audit))
