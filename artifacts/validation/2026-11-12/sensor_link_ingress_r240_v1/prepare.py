from pathlib import Path
import hashlib,json,zipfile
A=Path(__file__).resolve().parent;root=A.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
(A/'.gitignore').write_text('run/\nbuild/*.bin\nbuild/symbols.txt\n',encoding='utf-8')
(A/'run.sh').write_text('''#!/usr/bin/env bash
set -eo pipefail
cd /home/q/it_arena_sensor_link_ingress_r240_20261112/worktree
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:/home/q/it_arena_imu_20261111/tools_venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
/usr/bin/python3 -B artifacts/validation/2026-11-12/sensor_link_ingress_r240_v1/run.py
''',encoding='utf-8',newline='\n')
files=set()
for name in ('arena_autonomy','arena_vehicle_interface'):
    files.update((root/'src'/name/name).glob('*.py'))
files.update(root/'scripts'/name for name in ('probe_esp32_imu.py','bridge_sensor_link_preview.py'))
files.update(A/name for name in ('run.py','receiver.py','flash.py','run.sh','prepare.py','build/arena_sensor_link.bin'))
manifest={str(p.relative_to(root)).replace('\\','/'):sha(p) for p in sorted(files)}
(A/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
(A/'bundle_receipt.json').write_text(json.dumps(dict(firmware_sha256=sha(A/'build/arena_sensor_link.bin')),indent=2),encoding='utf-8')
files.update((A/'source_manifest.json',A/'bundle_receipt.json'))
dest=root/'build/r240_bundle.zip'
with zipfile.ZipFile(dest,'w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(files):z.write(p,p.relative_to(root))
(A/'upload_receipt.json').write_text(json.dumps(dict(sha256=sha(dest),bytes=dest.stat().st_size,files=len(files)),indent=2)+'\n',encoding='utf-8')
print((A/'upload_receipt.json').read_text())
