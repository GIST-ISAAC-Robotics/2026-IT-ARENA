"""현재 공통 입력 소스와 r161의 무구동 ROS 수신기를 새 격리 폴더로 묶는다."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parents[4]
A = Path(__file__).resolve().parent
before = A.parent/'imu_ros_preview_r161_v1'
(A/'receiver.py').write_bytes((before/'receiver.py').read_bytes())
s = (before/'run.py').read_text(encoding='utf-8')
s = s.replace("A=Path(__file__)", "assert os.environ.get('ROS_DOMAIN_ID')=='190' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'\nA=Path(__file__)")
(A/'run.py').write_text(s, encoding='utf-8')
(A/'run.sh').write_text('''#!/usr/bin/env bash
set -eo pipefail
cd /home/q/it_arena_imu_restore_r237_20261112/worktree
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=190 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:/home/q/it_arena_imu_20261111/tools_venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
/usr/bin/python3 -B artifacts/validation/2026-11-12/imu_restore_r237_v1/run.py
''', encoding='utf-8', newline='\n')
files = set()
for package in ('arena_autonomy', 'arena_vehicle_interface'):
    files.update((ROOT/'src'/package/package).rglob('*.py'))
files.update(ROOT/'scripts'/name for name in ('probe_esp32_imu.py','bridge_esp32_imu_preview.py','audit_imu_probe_recording.py'))
files.update(A/name for name in ('receiver.py','run.py','run.sh','prepare_capture.py'))
manifest = {str(path.relative_to(ROOT)).replace('\\','/'): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(files)}
(A/'capture_source_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
files.add(A/'capture_source_manifest.json')
target = ROOT/'build/r237_capture.zip'
with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED) as bundle:
    for path in sorted(files):bundle.write(path,path.relative_to(ROOT))
receipt = dict(path='build/r237_capture.zip',sha256=hashlib.sha256(target.read_bytes()).hexdigest(),bytes=target.stat().st_size,files=len(files))
(A/'capture_bundle_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8')
print(json.dumps(receipt))
