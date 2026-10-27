"""고정 E1 입력 보존. 검사 중 새 무관 파일은 기존 소스 변경으로 세지 않는다."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
MANIFEST = OUT / 'run8_source_manifest.json'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


if '--check' in sys.argv:
    original = json.loads(MANIFEST.read_text())
    changed = [entry['path'] for entry in original
               if not (ROOT / entry['path']).is_file()
               or digest(ROOT / entry['path']) != entry['sha256']]
    print(json.dumps({'passed': not changed, 'changed': changed, 'files': len(original)}))
    raise SystemExit(bool(changed))

if MANIFEST.exists():
    raise FileExistsError('기존 E1 스냅샷은 덮어쓰지 않습니다.')
paths = sorted(p for folder in ('src', 'scripts', 'tests', 'config')
               for p in (ROOT / folder).rglob('*')
               if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ('.pyc', '.pyo'))
MANIFEST.write_text(json.dumps([{'path': p.relative_to(ROOT).as_posix(), 'sha256': digest(p)}
                                for p in paths], indent=2))
with zipfile.ZipFile(OUT / 'run8_source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
    for path in paths:
        archive.write(path, path.relative_to(ROOT).as_posix())
print(json.dumps({'files': len(paths), 'passed': True}))
