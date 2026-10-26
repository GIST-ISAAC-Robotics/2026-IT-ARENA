"""선택형 후보의 실행/회귀 소스를 덮어쓰기 없이 보존."""
import hashlib
import inspect
import json
from pathlib import Path
import sys
import zipfile

out = Path(__file__).resolve().parent
root = out.parents[3]
sys.path.insert(0, str(root/'scripts'))
from validate_g1_traffic import source_paths, CONFIG
import arena_autonomy.r0_plus as candidate

digest = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert digest(inspect.getsourcefile(candidate)) == digest(root/'src/arena_autonomy/arena_autonomy/r0_plus.py')
paths = sorted(set(source_paths(CONFIG)+[
    'tests/test_r0_plus.py', 'tests/test_r0_plus_integration.py', 'tests/test_follow_trace.py',
    'scripts/profile_follow_trace.py',
    'config/tests/e3_two_actors_harness_v1.json', 'config/tests/e3_stop_restart_v1.json']))
with (out/'source_manifest.json').open('x') as stream:
    json.dump({p: digest(root/p) for p in paths}, stream, indent=2)
with zipfile.ZipFile(out/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
    for p in paths:
        archive.write(root/p, p)
print(json.dumps({'files': len(paths), 'installed_candidate_matches': True}))
