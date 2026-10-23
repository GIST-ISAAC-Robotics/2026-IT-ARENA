"""마감 시험의 소스 보존·불변성 검사. 실행/설정은 수정하지 않는다."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(name, value):
    with (OUT/name).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False)
        stream.write('\n')


def main():
    if sys.argv[1:] == ['before']:
        prior = []
        for name in ('2026-10-15/g1_corridor_integration_v1', '2026-10-16/g1_corridor_solo_v1'):
            path = ROOT/'artifacts/validation'/name/'pre_run_manifest.json'
            manifest = json.loads(path.read_text())
            changed = [row['path'] for row in manifest
                       if sha(ROOT/row['path'].replace('\\', '/')) != row['sha256']]
            relevant = [p for p in changed if p.replace('\\', '/').split('/')[0] in ('src', 'scripts', 'tests', 'config')]
            prior.append(dict(manifest=str(path.relative_to(ROOT)), files=len(manifest),
                              changed_existing_files=changed, changed_execution_sources=relevant))
        paths = {p for name in ('src', 'scripts', 'tests', 'config')
                 for p in (ROOT/name).rglob('*') if p.is_file() and '__pycache__' not in p.parts
                 and p.suffix not in ('.pyc', '.pyo')}
        paths.update(OUT/name for name in ('README.md', 'check.py', 'run.sh'))
        paths.update(ROOT/name for name in ('.gitignore', '.gitattributes'))
        manifest = [dict(path=p.relative_to(ROOT).as_posix(), sha256=sha(p)) for p in sorted(paths)]
        save('pre_run_manifest.json', manifest)
        with zipfile.ZipFile(OUT/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(paths):
                archive.write(path, path.relative_to(ROOT).as_posix())
        passed = all(not row['changed_execution_sources'] for row in prior)
        save('prior_integrity.json', dict(prior=prior, passed=passed))
        print(json.dumps(dict(prior_execution_sources_unchanged=passed, snapshot_files=len(manifest))))
        return 0 if passed else 2
    if sys.argv[1:] == ['after']:
        manifest = json.loads((OUT/'pre_run_manifest.json').read_text())
        changed = [row['path'] for row in manifest if sha(ROOT/row['path']) != row['sha256']]
        result = dict(files=len(manifest), changed=changed, passed=not changed)
        save('post_run_integrity.json', result)
        print(json.dumps(result))
        return 0 if result['passed'] else 2
    raise ValueError('choose before or after')


if __name__ == '__main__':
    raise SystemExit(main())
