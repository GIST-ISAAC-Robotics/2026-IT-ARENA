"""기존 SHM 16 MiB 진단 경로만 별도 시험 stage에 적용한다."""
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
C=A/'candidate_shm16'
C.mkdir(exist_ok=False)
for name in ('analyze.py','causal_analysis.py','node_runner.py','run_replay.py'):
    shutil.copyfile(A/name,C/name)
for name in ('profile.json','reconstructed_inputs.json.gz'):
    shutil.copyfile(A/'bundle'/name,C/name)
shutil.copyfile(ROOT/'scripts/replay_transport.py',C/'replay_transport.py')
p=C/'run_replay.py'
s=p.read_text(encoding='utf8')
s=s.replace("    import rclpy\n", "    from replay_transport import configure, audit_current\n"
    "    probe=Path('/home/q/it_arena_a3_20260928/build/libarena_dds_probe_v1.so')\n"
    "    assert SHA(probe)=='b235a5df0f8abe0c0c014209e687578d1558bdd9a587c61fb5355c982982adbf'\n"
    "    write(out/'dds_requested.json',configure('shm_16m',probe,out))\n"
    "    import rclpy\n",1)
s=s.replace("    executor = SingleThreadedExecutor()", "    audit_current('input_observer')\n    executor = SingleThreadedExecutor()",1)
s=s.replace("        # Both sides of each input/output edge", "        for role in children:\n"
    "            assert (out/('dds_active_'+role+'.json')).is_file(),role\n"
    "        # Both sides of each input/output edge",1)
p.write_text(s,encoding='utf8')
p=C/'node_runner.py'
s=p.read_text(encoding='utf8').replace("        summary.update(role=args.role", "        from replay_transport import audit_current\n"
    "        audit_current(args.role)\n"
    "        summary.update(role=args.role",1)
p.write_text(s,encoding='utf8')
manifest=json.loads((A/'manifest.json').read_bytes())
manifest.update(harness_revision=3,candidate='existing explicit shm_16m transport; unchanged controller/guard/QoS/TTL',
    parent_baseline_result='remote_result/result',
    dds_probe_sha256='b235a5df0f8abe0c0c014209e687578d1558bdd9a587c61fb5355c982982adbf')
manifest['files']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(C.iterdir()) if p.is_file()}
(C/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
for p in C.glob('*.py'):compile(p.read_bytes(),str(p),'exec')
with zipfile.ZipFile(A/'candidate_shm16.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in C.iterdir():z.write(p,p.name)
print(json.dumps(dict(files=len(manifest['files']),zip_sha256=hashlib.sha256((A/'candidate_shm16.zip').read_bytes()).hexdigest())))
