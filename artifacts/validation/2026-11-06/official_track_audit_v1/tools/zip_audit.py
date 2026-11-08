"""릴리스 ZIP 목록·경로 안전성 검사, 빈 새 경로 추출, main 트리·이전 ZIP과 바이트 대조.

사용: python -I tools/zip_audit.py  (저장소 루트 기준 경로를 내부에서 계산)
"""
from pathlib import Path, PurePosixPath
import hashlib
import json
import stat
import sys
import zipfile

ART = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parents[5]
ZIP = ROOT / 'assets/track/official/v2026.11.06/it_arena_track_v2026.11.06.zip'
OLD_ZIP = ROOT / 'assets/track/official/v2026.11.01/it_arena_track_v2026.11.01.zip'
EXTRACT = ART / 'extracted'
TREE = ART / 'upstream/api/tree_main.json'

# 공식 README/MANUAL이 배포본에 포함된다고 서술하거나 시험에 필요한 항목.
# ZIP은 공식 track/ 폴더 기준 상대 경로다(최상위 접두 폴더 없음).
REQUIRED = ['output_final/' + n for n in (
    'world.sdf', 'scene.json', 'centerline.csv', 'left_boundary.csv', 'right_boundary.csv',
    'branch_0.csv', 'branch_1.csv', 'map.png', 'map.yaml', 'map_with_grass.png',
    'map_with_grass.yaml', 'traffic_light.py', 'aruco/aruco_id0.png', 'aruco/aruco_id20.png',
    'aruco/aruco_id30.png', 'aruco/aruco_id45.png')] + ['README.md', 'track_610mm_build_2026-11-05.pdf']


def sha256(b):
    return hashlib.sha256(b).hexdigest()


def blob(b):
    return hashlib.sha1(b'blob %d\0' % len(b) + b).hexdigest()


def safe_name(name):
    p = PurePosixPath(name)
    problems = []
    if name.startswith('/') or name.startswith('\\') or (len(name) > 1 and name[1] == ':'):
        problems.append('absolute')
    if '..' in p.parts:
        problems.append('dotdot')
    if '\\' in name:
        problems.append('backslash')
    if any(ord(c) < 32 for c in name):
        problems.append('control-char')
    return problems


def main():
    zf = zipfile.ZipFile(ZIP)
    infos = zf.infolist()
    entries, issues = [], []
    for i in infos:
        mode = (i.external_attr >> 16) & 0xFFFF
        is_link = stat.S_ISLNK(mode)
        prob = safe_name(i.filename) + (['symlink'] if is_link else [])
        if prob:
            issues.append({'name': i.filename, 'problems': prob})
        entries.append({'name': i.filename, 'size': i.file_size, 'compressed': i.compress_size,
                        'is_dir': i.is_dir(), 'crc': f'{i.CRC:08x}', 'date_time': list(i.date_time),
                        'mode_octal': oct(mode), 'flag_bits': i.flag_bits})
    names = [i.filename for i in infos]
    dup = sorted({n for n in names if names.count(n) > 1})
    total_unc = sum(i.file_size for i in infos)
    bad_crc = zf.testzip()
    result = {'zip': str(ZIP.relative_to(ROOT)).replace('\\', '/'), 'zip_bytes': ZIP.stat().st_size,
              'zip_sha256': sha256(ZIP.read_bytes()), 'entries': len(infos),
              'file_entries': sum(not i.is_dir() for i in infos), 'uncompressed_total': total_unc,
              'path_issues': issues, 'duplicates': dup, 'testzip_first_bad': bad_crc}
    if issues or dup or bad_crc:
        result['extracted'] = False
        (ART / 'checks').mkdir(exist_ok=True)
        (ART / 'checks/zip_audit.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8', newline='\n')
        print('UNSAFE ZIP, not extracted', file=sys.stderr)
        return 2
    # 빈 새 경로에만 추출한다.
    # 재실행 때는 기존 추출본을 덮어쓰지 않고 바이트 일치만 확인한다.
    reuse = EXTRACT.exists() and any(EXTRACT.iterdir())
    EXTRACT.mkdir(parents=True, exist_ok=True)
    root_resolved = EXTRACT.resolve()
    for i in infos:
        if reuse:
            if not i.is_dir() and (EXTRACT / i.filename).read_bytes() != zf.read(i):
                raise RuntimeError('extracted copy differs: ' + i.filename)
            continue
        target = (EXTRACT / i.filename).resolve()
        if root_resolved not in target.parents and target != root_resolved:
            raise RuntimeError('escape ' + i.filename)
        if i.is_dir():
            target.mkdir(parents=True, exist_ok=True)
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        data = zf.read(i)
        target.write_bytes(data)
    # 최상위 접두 폴더 추정
    tops = sorted({n.split('/')[0] for n in names})
    result['top_level'] = tops
    prefix = tops[0] + '/' if len(tops) == 1 and any('/' in n for n in names) else ''
    files = {}
    for i in infos:
        if i.is_dir():
            continue
        data = zf.read(i)
        rel = i.filename[len(prefix):] if prefix and i.filename.startswith(prefix) else i.filename
        files[rel] = {'bytes': len(data), 'sha256': sha256(data), 'git_blob': blob(data)}
    result['prefix'] = prefix
    result['extract_reused_verified'] = reuse
    result['files'] = files
    # main track/ 와 대조 (track/output_final/ 또는 track/)
    tree = json.loads(TREE.read_text(encoding='utf-8'))
    blobs = {e['path']: e['sha'] for e in tree['tree'] if e['type'] == 'blob'}
    vs_main = []
    for rel, rec in files.items():
        cands = [p for p in (f'track/output_final/{rel}', f'track/{rel}', rel) if p in blobs]
        hit = next((p for p in cands if blobs[p] == rec['git_blob']), None)
        vs_main.append({'zip_path': rel, 'main_path': hit or (cands[0] if cands else None),
                        'same_bytes_as_main': hit is not None})
    result['vs_main_45691aec'] = vs_main
    out_main = sorted(p for p in blobs if p.startswith('track/output_final/'))
    zipped = {r['main_path'] for r in vs_main if r['main_path']}
    result['main_output_final_not_in_zip'] = [p for p in out_main if p not in zipped]
    result['main_track_other_not_in_zip'] = sorted(p for p in blobs if p.startswith('track/') and not p.startswith('track/output_final/') and p not in zipped)
    # 필수 항목
    result['required_missing'] = [r for r in REQUIRED if r not in files]
    # 이전 v2026.11.01 ZIP과 비교
    old = zipfile.ZipFile(OLD_ZIP)
    onames = [n for n in old.namelist() if not n.endswith('/')]
    otops = sorted({n.split('/')[0] for n in onames})
    oprefix = otops[0] + '/' if len(otops) == 1 else ''
    ofiles = {n[len(oprefix):]: sha256(old.read(n)) for n in onames}
    result['vs_v2026_11_01_zip'] = {
        'old_prefix': oprefix,
        'same': sorted(k for k in files if k in ofiles and ofiles[k] == files[k]['sha256']),
        'changed': sorted(k for k in files if k in ofiles and ofiles[k] != files[k]['sha256']),
        'added': sorted(k for k in files if k not in ofiles),
        'removed': sorted(k for k in ofiles if k not in files)}
    (ART / 'checks').mkdir(exist_ok=True)
    (ART / 'checks/zip_audit.json').write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8', newline='\n')
    summary = {k: result[k] for k in ('entries', 'file_entries', 'uncompressed_total', 'path_issues', 'duplicates',
                                      'testzip_first_bad', 'top_level', 'prefix', 'required_missing',
                                      'main_output_final_not_in_zip', 'main_track_other_not_in_zip')}
    summary['not_same_as_main'] = [r for r in vs_main if not r['same_bytes_as_main']]
    summary['vs_old'] = result['vs_v2026_11_01_zip']
    print(json.dumps(summary, indent=1, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
