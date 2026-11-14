import hashlib,json,shutil,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent
C=A/'candidate_shm16_drain'
C.mkdir(exist_ok=False)
for p in (A/'candidate_shm16').iterdir():
    if p.is_file() and p.name!='manifest.json':shutil.copyfile(p,C/p.name)
for name in ('motion_ingress_candidate.py','test_motion_ingress_candidate.py'):
    shutil.copyfile(A/name,C/name)
p=C/'node_runner.py';s=p.read_text(encoding='utf8')
s=s.replace('        def control(self):\n            began=time.monotonic_ns()',
    '        def control(self):\n'
    '            from motion_ingress_candidate import drain_ready\n'
    '            before_drain={k:list(getattr(self.motion.history,k).times[-1:]) for k in ("wheels","gyro")}\n'
    '            drained=drain_ready(self._pending_motion_subscriptions)\n'
    '            began=time.monotonic_ns()',1)
s=s.replace('record=dict(begin_monotonic_ns=began,begin_clock_s=clock)',
    'record=dict(begin_monotonic_ns=began,begin_clock_s=clock,pending_drain=drained,source_before_drain=before_drain)',1)
s=s.replace("        from replay_transport import audit_current", 
    "        if hasattr(owner,'motion'):\n"
    "            owner._pending_motion_subscriptions=tuple(s for s in owner.subscriptions if s.msg_type.__name__ in ('JointState','Imu'))\n"
    "            assert len(owner._pending_motion_subscriptions)==2\n"
    "            assert {s.topic_name for s in owner._pending_motion_subscriptions}=={PREFIX+'/drive_motor/encoder',PREFIX+'/imu/data'}\n"
    "        from replay_transport import audit_current",1)
p.write_text(s,encoding='utf8')
manifest=json.loads((A/'candidate_shm16/manifest.json').read_bytes())
manifest.update(harness_revision=4,candidate='shm_16m plus bounded ready-motion drain at control entry; unchanged parsers/history/QoS/TTL')
manifest['files']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(C.iterdir()) if p.is_file()}
(C/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
for p in C.glob('*.py'):compile(p.read_bytes(),str(p),'exec')
with zipfile.ZipFile(A/'candidate_shm16_drain.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in C.iterdir():z.write(p,p.name)
print(json.dumps(dict(files=len(manifest['files']),zip_sha256=hashlib.sha256((A/'candidate_shm16_drain.zip').read_bytes()).hexdigest())))
