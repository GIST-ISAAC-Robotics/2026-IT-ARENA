"""실행 전 소스/설치 라이브러리 해시 보존. 기존 자료 덮어쓰기 금지."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import rclpy.clock

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
files=['src/arena_vehicle_interface/arena_vehicle_interface/clock_pair_audit.py',
       'src/arena_vehicle_interface/arena_vehicle_interface/clock_bounds.py',
       'scripts/audit_clock_pairs.py','scripts/capture_clock_pairs.py','scripts/audit_clock_exchanges.py',
       'tests/test_clock_pair_audit.py',str(Path(__file__).relative_to(ROOT)),
       str((BASE/'README.md').relative_to(ROOT)),str((BASE/'run.sh').relative_to(ROOT))]
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
hashes={p:digest(ROOT/p) for p in files}
with zipfile.ZipFile(BASE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(ROOT/p,p)
with (BASE/'source_manifest.json').open('x') as stream:
    json.dump(dict(source_sha256=hashes,python=sys.version,
        installed_rclpy_clock=dict(path=rclpy.clock.__file__,sha256=digest(Path(rclpy.clock.__file__)))),
        stream,indent=2)
