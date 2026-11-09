"""r52의 미실행 D launch 거부 수정. 앞선 시험과 계약을 덮어쓰지 않는다."""
import hashlib
import json
import zipfile
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
parent = 'config/tests/signal_posts_markers_r52_sha256.json'
target = 'src/arena_bringup/launch/local_pursuit.launch.py'
contract = ROOT/'config/tests/signal_posts_marker_launch_r53_sha256.json'
snapshot = A/'before.zip'
sha = lambda b: hashlib.sha256(b).hexdigest()
assert not contract.exists() and not snapshot.exists()
old = (A/'local_pursuit.launch.before.py').read_bytes()
with zipfile.ZipFile(snapshot, 'x', zipfile.ZIP_DEFLATED) as z:
    z.writestr(target, old)
record = dict(schema_version=1, scope='r53: accept blank_20_30 evaluation mode at top-level local pursuit launch; controller unchanged',
              parent_contract=parent, parent_contract_sha256=sha((ROOT/parent).read_bytes()),
              before_snapshot=snapshot.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(snapshot.read_bytes()),
              approved_changes={target: dict(before=sha(old), after=sha((ROOT/target).read_bytes()),
                                reason='Fix omitted top-level launch choice; D previously ended before simulation')},
              approved_evaluation_scripts={}, new_dependencies={})
contract.write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
print('registered r53; no simulator started')
