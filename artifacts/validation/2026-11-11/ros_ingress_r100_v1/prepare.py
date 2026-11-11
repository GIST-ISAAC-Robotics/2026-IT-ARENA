"""r98 동결 자료와 새 진단/프로세스 파일만 전송 묶음으로 보존한다."""
from pathlib import Path
import hashlib
import io
import json
import tarfile
import time

ROOT=Path(__file__).resolve().parents[4]
A=Path(__file__).resolve().parent
sha=lambda data:hashlib.sha256(data).hexdigest()
parent='artifacts/validation/2026-11-11/control_10hz_cost_r98_v1/bundle_manifest.json'
manifest=json.loads((ROOT/parent).read_bytes())
assert all(sha((ROOT/n).read_bytes())==v['sha256'] for n,v in manifest['files'].items())
files=list(manifest['files'])+[parent,'src/arena_autonomy/arena_autonomy/decision_process.py',
    'artifacts/validation/2026-11-11/ros_ingress_r100_v1/probe.py']
archive=ROOT/'build/ros_ingress_r100_20261111.tar.gz'
assert not archive.exists() and not (A/'bundle_manifest.json').exists()
inventory={}
with tarfile.open(archive,'x:gz',compresslevel=1) as out:
    for name in files:
        data=(ROOT/name).read_bytes();item=tarfile.TarInfo(name)
        item.size,item.mtime=len(data),int(time.time());out.addfile(item,io.BytesIO(data))
        inventory[name]=dict(size=len(data),sha256=sha(data))
result=dict(files=inventory,archive_sha256=sha(archive.read_bytes()),archive_size=archive.stat().st_size,
    scope='localhost ROS ingress under recorded compute; no actuation or installed package changes')
(A/'bundle_manifest.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in result.items() if k!='files'}))
