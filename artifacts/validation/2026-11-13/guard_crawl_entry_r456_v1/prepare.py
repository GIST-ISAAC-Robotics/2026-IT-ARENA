"""r452 동결 원본에서 독립 r456 검토 사본을 만든다. 공통 입력은 읽기만 한다."""
import hashlib
import json
from pathlib import Path
import zipfile

B=Path(__file__).resolve().parent
SOURCE=B.parent/'guard_crawl_drive_r452_v1'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert not (B/'base').exists()
with zipfile.ZipFile(SOURCE/'sources.zip') as z:
    selected=[n for n in z.namelist() if n.startswith(('src/arena_autonomy/arena_autonomy/',
        'src/arena_vehicle_interface/arena_vehicle_interface/')) or n in (
        'tests/test_runtime_guard_crawl.py','tests/test_runtime_released_plan.py')]
    for name in selected:
        target=B/'base'/name
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes(z.read(name))
manifest={str(p.relative_to(B)):sha(p) for p in (B/'base').rglob('*') if p.is_file()}
manifest.update({str(p.relative_to(B.parent)):sha(p) for p in [SOURCE/'sources.zip',SOURCE/'run1/follow_decisions.jsonl']})
(B/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(dict(base_files=len(selected),r452_archive_sha256=sha(SOURCE/'sources.zip'),trace_sha256=sha(SOURCE/'run1/follow_decisions.jsonl'))))
