#!/usr/bin/env python3
"""경기 종료 보존 목록 생성. 파일 삭제·장치 접근·Git 변경은 하지 않는다."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = 'artifacts/validation/2026-11-14/repository_closeout_v1'


def git_paths(*args):
    data = subprocess.check_output(['git', *args, '-z'], cwd=ROOT)
    return {name.decode('utf-8') for name in data.split(b'\0') if name}


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8', newline='\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--before-list', type=Path, required=True,
                    help='정리 시작 때 보존한 git ls-files --others --exclude-standard JSON 목록')
    args = ap.parse_args()
    out = ROOT / OUT
    out.mkdir(parents=True, exist_ok=True)
    tracked = git_paths('ls-files')
    candidates = git_paths('ls-files', '--others', '--exclude-standard')
    published = tracked | candidates
    groups = defaultdict(lambda: {'files': 0, 'bytes': 0, 'reports': [], 'manifests': []})
    for name in sorted(published):
        p = Path(name)
        if not p.parts or p.parts[0] not in ('artifacts', 'assets', 'outputs') or name.startswith(OUT + '/'):
            continue
        absolute = ROOT / p
        if not absolute.is_file() or absolute.is_symlink():
            continue
        depth = 4 if p.parts[0] == 'artifacts' else min(3, len(p.parts)-1)
        key = '/'.join(p.parts[:depth])
        g = groups[key]
        g['files'] += 1
        g['bytes'] += absolute.stat().st_size
        if p.suffix.lower() == '.md':
            g['reports'].append(name)
        if p.suffix.lower() == '.json' and any(s in p.name.lower() for s in ('manifest', 'receipt', 'registration')):
            g['manifests'].append(name)
    save(out / 'artifact_index.json', {
        'schema': 'arena_closeout_artifact_index_v1', 'date': '2026-11-14',
        'scope': 'public tracked and selected project artifacts; not a complete disk backup',
        'status_inferred_from_directory_name': False,
        'groups': [{'path': k, **v} for k, v in sorted(groups.items())]})

    # Newly excluded artifacts remain on disk. Omit local credentials/environment contents.
    before = set(json.loads(args.before_list.read_text(encoding='utf-8')))
    excluded = []
    for name in sorted(before - published):
        p = Path(name)
        if p.parts[0] not in ('artifacts', 'outputs', 'output'):
            continue
        absolute = ROOT / p
        if not absolute.is_file() or absolute.is_symlink():
            continue
        dependency = 'pydeps' in p.parts or 'wheels' in p.parts or '__pycache__' in p.parts
        row = {'path': name, 'bytes': absolute.stat().st_size,
               'reason': 'reinstallable_dependency' if dependency else 'local_raw_or_large_artifact',
               'preserved_on_disk': True}
        if not dependency:
            row['sha256'] = digest(absolute)
        excluded.append(row)
    save(out / 'local_only_manifest.json', {
        'schema': 'arena_closeout_local_only_v1',
        'scope': 'newly excluded project files only; older ignored files and full disk are not enumerated',
        'deleted_files': 0, 'files': excluded})

    final_bundles = []
    for name, status in [
        ('artifacts/validation/2026-11-14/runtime_update_handover_r533_v1/selected_v2/runtime_handover_sources.zip', 'final_esp32_source_only'),
        ('build/uno_q_deployment_20261114_v3/uno_q_deployment.zip', 'unused_alternative_local_only'),
        ('build/uno_q_firmware_20261114/unconfigured/zephyr.bin', 'unused_alternative_local_only'),
        ('build/uno_q_firmware_20261114/bench/zephyr.bin', 'unused_alternative_local_only')]:
        p = ROOT / name
        final_bundles.append({'path': name, 'status': status, 'exists': p.is_file(),
                              **({'bytes': p.stat().st_size, 'sha256': digest(p)} if p.is_file() else {})})
    save(out / 'bundle_index.json', {'bundles': final_bundles, 'hardware_deployed': False})
    print(json.dumps({'public_groups': len(groups), 'new_local_only_files': len(excluded),
                      'new_local_only_bytes': sum(x['bytes'] for x in excluded),
                      'files_deleted': 0}, ensure_ascii=False))


if __name__ == '__main__':
    main()
