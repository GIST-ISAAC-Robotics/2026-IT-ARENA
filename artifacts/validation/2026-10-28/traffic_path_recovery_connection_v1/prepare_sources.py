"""직전 실패 실행의 소스와 일치하는 이전 바이트를 새 연결 전에 보존한다."""
import hashlib
import json
from pathlib import Path
import zipfile
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
paths=[
 'src/arena_autonomy/arena_autonomy/local_follow.py',
 'src/arena_autonomy/arena_autonomy/corridor_observer.py',
 'src/arena_autonomy/arena_autonomy/local_pursuit.py',
 'src/arena_bringup/launch/demo.launch.py',
 'src/arena_bringup/launch/local_pursuit.launch.py',
 'scripts/validate_g1_traffic.py','scripts/profile_follow_trace.py',
 'scripts/prepare_jetson_bundle.py','scripts/validate_pipeline_faults_ros.py']
original=json.loads((BASE.parent/'traffic_release_fringe_v1/run1_source_manifest.json').read_text())
expected={r['path']:r['sha256'] for r in original}
sha=lambda data:hashlib.sha256(data).hexdigest()
before={p:sha((ROOT/p).read_bytes()) for p in paths}
assert all(before[p]==expected[p] for p in paths)
with zipfile.ZipFile(BASE/'before.zip','x',zipfile.ZIP_DEFLATED) as archive:
    for p in paths:archive.write(ROOT/p,p)
(BASE/'before_sha256.json').write_text(json.dumps(before,indent=2),encoding='utf-8')
print(json.dumps(dict(files=len(paths),matches_previous_run=True)))
