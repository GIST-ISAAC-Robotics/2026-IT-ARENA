"""E3 3대 시험기 실행 직전 입력·코드·단위 회귀 보존."""
import json
from pathlib import Path
import sys
import zipfile

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from validate_g1_traffic import source_paths, sha256, CONFIG

overlay = ROOT/'config/tests/e3_two_actors_harness_v1.json'
paths = sorted(set(source_paths(CONFIG, actor_overlay=overlay)+[
    'tests/test_traffic_actors.py', 'tests/test_traffic_actor_runtime.py', 'tests/test_traffic_actor_integration.py']))
if (OUT/'source_manifest.json').exists():
    raise FileExistsError('existing E3 snapshot')
(OUT/'source_manifest.json').write_text(json.dumps({p: sha256(ROOT/p) for p in paths}, indent=2))
with zipfile.ZipFile(OUT/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
    for path in paths:
        archive.write(ROOT/path, path)
print(json.dumps({'files': len(paths)}))
