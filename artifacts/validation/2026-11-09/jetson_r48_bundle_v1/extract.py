"""검사 통과한 archive를 build/ 하위의 새 격리 폴더에만 추출하고 추출 파일을 manifest와 대조한다. python3 -I -B."""
import hashlib
import json
from pathlib import Path
import sys
import tarfile

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
ARCHIVE = REPO/'build/jetson_follow_20261109_r48.tar.gz'
TARGET = (REPO/'build/jetson_follow_20261109_r48').resolve()


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


def main():
    assert (REPO/'AGENTS.md').is_file(), REPO
    if not TARGET.is_absolute() or not TARGET.is_relative_to((REPO/'build').resolve()) or TARGET == (REPO/'build').resolve():
        raise ValueError('extract target must be a new folder under workspace build/')
    if TARGET.exists():
        raise FileExistsError('existing extract folder is not reused or overwritten')
    verify = json.loads((HERE/'verify_result.json').read_text(encoding='utf-8'))
    if not verify['passed']:
        raise ValueError('bundle verification did not pass')
    manifest = json.loads((HERE/'bundle_manifest.json').read_text(encoding='utf-8'))
    TARGET.mkdir(exist_ok=False)
    with tarfile.open(ARCHIVE, 'r:gz') as archive:
        archive.extractall(TARGET, filter='data')
    found = {p.relative_to(TARGET).as_posix(): p for p in TARGET.rglob('*') if p.is_file()}
    mismatch = sorted(n for n, meta in manifest['files'].items()
                      if n not in found or found[n].stat().st_size != meta['size'] or digest(found[n]) != meta['sha256'])
    result = dict(target=TARGET.relative_to(REPO).as_posix(), files=len(found), manifest_files=len(manifest['files']),
                  extra=sorted(set(found)-set(manifest['files'])), mismatch=mismatch)
    result['passed'] = not result['extra'] and not mismatch and len(found) == len(manifest['files'])
    with (HERE/'extract_result.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
