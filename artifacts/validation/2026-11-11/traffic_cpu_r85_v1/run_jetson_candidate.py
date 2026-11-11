"""새 격리 디렉터리에서 r82+단일 CPU 변경을 계측. 실물 구동/환경 변경 없음."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile

root=Path('/home/q/it_arena_traffic_r85_cpu_20261111')
archive=Path('/home/q/it_arena_traffic_r85_cpu_20261111.tar.gz')
manifest_path=Path('/home/q/it_arena_traffic_r85_cpu_manifest_20261111.json')
measure=Path('/home/q/r85_measure_candidate_20261111.py')
output=Path('/home/q/r85_candidate_cost_20261111.json')
manifest=json.loads(manifest_path.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert not output.exists() and sha(archive)==manifest['archive_sha256']
root.mkdir(exist_ok=False)
with tarfile.open(archive,'r:gz') as bundle:
    for member in bundle:
        path=(root/member.name).resolve()
        assert member.isfile() and path.is_relative_to(root) and member.name in manifest['files']
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes(bundle.extractfile(member).read())
assert all(sha(root/n)==v['sha256'] for n,v in manifest['files'].items())
env=os.environ.copy()
env['PYTHONPATH']=':'.join(str(root/n) for n in ('src/arena_autonomy','src/arena_vehicle_interface','scripts'))
result=subprocess.run([sys.executable,'-B',str(measure),'--trace',
    str(root/'artifacts/r82_expected/follow_decisions.jsonl'),
    '--manifest',str(manifest_path),'--out',str(output)],cwd=root,env=env)
raise SystemExit(result.returncode)
