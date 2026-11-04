import json,hashlib,glob
from pathlib import Path
for f in sorted(glob.glob('artifacts/validation/2026-11-04/lidar_mount_performance_v1/*_registered.json')):
    d=json.load(open(f,encoding='utf-8'))
    bad=[p for p,h in d['source_sha256'].items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
    print(Path(f).name, len(d['source_sha256']), 'changed_since:', bad)
