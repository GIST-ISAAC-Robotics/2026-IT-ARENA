"""승인된 한 번의 시험 전 소스 불변성과 후속 계약을 확인하고 증거를 보존한다."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if (OUT / 'pre_run_manifest.json').exists():
        raise FileExistsError('이미 준비된 시험을 덮어쓰지 않습니다.')
    old = json.loads((ROOT / 'artifacts/validation/2026-10-15/g1_corridor_integration_v1/pre_run_manifest.json').read_text())
    changed = [item['path'] for item in old
               if digest(ROOT / item['path'].replace('\\', '/')) != item['sha256']]
    contract = json.loads((ROOT / 'config/tests/corridor_fallback_r0_sha256.json').read_text())
    checks = {'prior_snapshot_unchanged': not changed,
              'parent_contract': digest(ROOT / contract['parent_contract']) == contract['parent_contract_sha256']}
    for group in ('approved_changes', 'approved_evaluation_scripts'):
        for path, values in contract[group].items():
            checks[path] = digest(ROOT / path) == values['after']
    for path, value in contract['new_dependencies'].items():
        checks[path] = digest(ROOT / path) == value
    report = {'prior_manifest_files': len(old), 'changed': changed, 'checks': checks,
              'passed': all(checks.values())}
    (OUT / 'preflight_integrity.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    if not report['passed']:
        raise ValueError('사전 소스 계약과 다릅니다. 재검토가 필요합니다.')
    paths = []
    for folder in ('src', 'scripts', 'tests', 'config'):
        paths.extend(p for p in (ROOT / folder).rglob('*') if p.is_file()
                     and '__pycache__' not in p.parts and p.suffix not in ('.pyc', '.pyo'))
    paths.extend(OUT / name for name in ('README.md', 'run.sh', 'prepare.py'))
    paths.extend(ROOT / name for name in ('.gitignore', '.gitattributes'))
    paths = sorted(set(paths))
    manifest = [{'path': p.relative_to(ROOT).as_posix(), 'sha256': digest(p)} for p in paths]
    (OUT / 'pre_run_manifest.json').write_text(json.dumps(manifest, indent=2))
    with zipfile.ZipFile(OUT / 'pre_run_source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in paths:
            archive.write(path, path.relative_to(ROOT).as_posix())
    print(json.dumps({'passed': report['passed'], 'snapshot_files': len(manifest)}))


if __name__ == '__main__':
    main()
