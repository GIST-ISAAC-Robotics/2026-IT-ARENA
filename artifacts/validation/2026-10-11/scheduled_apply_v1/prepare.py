"""실행 전 작업 트리/기존 바이너리 보존. 덮어쓰기 금지."""
import hashlib,json,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
BASE=Path(__file__).resolve().parent
c=json.loads((ROOT/'config/tests/scheduled_command_r6_sha256.json').read_text())
files=set(c['sha256'])|set(c['implementation_sha256'])|set(c['evaluation_tools_sha256'])
files.update(['src/arena_vehicle_interface/setup.py','src/arena_vehicle_interface/arena_vehicle_interface/actuation_ros.py',
              'src/arena_gazebo/CMakeLists.txt','scripts/analyze_initial_repeat.py'])
with zipfile.ZipFile(BASE/'before_source.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(files):z.write(ROOT/p,p)
with (BASE/'before_manifest.json').open('x') as f:
    json.dump({p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(files)},f,indent=2)
binary=(ROOT/'install/arena_gazebo/lib/libarena-single-motor-system.so').read_bytes()
with (BASE/'before_plugin.so').open('xb') as f:f.write(binary)
print('before sources/binary preserved',hashlib.sha256(binary).hexdigest())
