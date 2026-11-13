"""r452 동결 사본에서 r459 독립 stage를 준비한다. 공통 소스는 변경하지 않는다."""
import hashlib
import json
from pathlib import Path
import shutil
B=Path(__file__).resolve().parent
base=B.parent/'guard_crawl_entry_r456_v1/base'
assert not (B/'stage').exists()
shutil.copytree(base,B/'stage')
paths=['src/arena_autonomy/arena_autonomy/'+p for p in ('pass_release_trial.py',
    'current_path_recheck/_present.py','current_path_recheck/_guard.py',
    'race_runtime_policy/retained_recheck/_present.py','race_runtime_policy/retained_recheck/_guard.py')]
manifest={name:hashlib.sha256((base/name).read_bytes()).hexdigest() for name in paths}
(B/'original_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(manifest))
