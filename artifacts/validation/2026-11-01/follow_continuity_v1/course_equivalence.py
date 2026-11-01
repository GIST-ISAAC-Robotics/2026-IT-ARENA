"""대표 코스 run1의 등록 입력·소스 사본과 현재 파일을 바이트로 대조한다(실행 없음)."""
from hashlib import sha256
import json
from pathlib import Path
import sys
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
RUN = BASE.parent / 'mixed_course_v1/run1'
REG = BASE.parent / 'mixed_course_v1/registered.json'


def digest(data):
    return sha256(data).hexdigest()


registered = json.loads(REG.read_bytes())['source_sha256']
same, changed, missing = [], [], []
for path, old in sorted(registered.items()):
    target = ROOT / path
    if not target.is_file():
        missing.append(path)
    elif digest(target.read_bytes()) == old:
        same.append(path)
    else:
        changed.append(path)
snapshot = {}
with zipfile.ZipFile(RUN / 'source_snapshot.zip') as archive:
    for name in archive.namelist():
        if name.endswith('/'):
            continue
        rel = name.split('source_snapshot/')[-1]
        snapshot[rel] = digest(archive.read(name))
snapshot_changed = sorted(p for p, h in snapshot.items()
                          if (ROOT / p).is_file() and digest((ROOT / p).read_bytes()) != h)
snapshot_missing = sorted(p for p in snapshot if not (ROOT / p).is_file())
now = {p.relative_to(ROOT).as_posix() for p in (ROOT / 'src/arena_autonomy/arena_autonomy').glob('*.py')}
new_sources = sorted(now - set(registered) - set(snapshot))
result = dict(registered_total=len(registered), registered_same=len(same), registered_changed=changed,
              registered_missing=missing, snapshot_total=len(snapshot), snapshot_changed=snapshot_changed,
              snapshot_missing=snapshot_missing, new_autonomy_sources=new_sources)
json.dump(result, sys.stdout, ensure_ascii=True, indent=1)
