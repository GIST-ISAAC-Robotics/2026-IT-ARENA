"""승인된 r466 22파일만 before SHA 확인 후 정확히 복사한다."""
import hashlib
import json
from pathlib import Path
import shutil

OUT = Path(__file__).resolve().parent
BUNDLE = OUT.parent
REPO = BUNDLE.parents[3]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def write(name, data):
    (OUT / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    assert not (OUT / 'integration.json').exists(), 'integration receipt already exists'
    rows = json.loads((BUNDLE / 'revision_v2/changes.json').read_text(encoding='utf-8'))
    checks = []
    for row in rows:
        target, staged = REPO / row['path'], BUNDLE / 'stage' / row['path']
        checks.append(dict(row, observed_before=sha(target), observed_stage=sha(staged)))
    write('before_check.json', checks)
    assert len(checks) == 22
    assert all(r['observed_before'] == r['original_sha256'] and
               r['observed_stage'] == r['stage_sha256'] for r in checks), 'before/stage SHA conflict'
    for row in checks:
        target = REPO / row['path']
        if target.exists():
            backup = OUT / 'originals' / row['path']
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(target, backup)
    for row in checks:
        target, staged = REPO / row['path'], BUNDLE / 'stage' / row['path']
        assert sha(target) == row['original_sha256'], 'changed since preflight'
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(staged, target)
        row['integrated_sha256'] = sha(target)
        assert row['integrated_sha256'] == row['stage_sha256']
    write('integration.json', {'status': 'integrated_pending_Windows_focus_tests',
        'files': checks, 'count': len(checks), 'conflicts': [],
        'source_manifest': '../revision_v2/changes.json',
        'WSL_used': False, 'Jetson_used': False, 'native_changed': False,
        'PROJECT_CONTEXT_or_DEVELOPMENT_PLAN_modified': False})
    print('INTEGRATED_EXACT 22/22; originals saved')
