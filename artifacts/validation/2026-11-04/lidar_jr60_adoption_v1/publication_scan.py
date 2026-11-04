"""게시 준비: 11/4 새 산출물 6개 폴더의 미추적·비무시 파일을 크기/종류별로 집계. git add 하지 않는다.

인자 없음. publication_scan.json 기록. 영상·원시 jsonl·대용량·토큰 형태 문자열은 별도 목록으로 표시한다.
"""
import collections
import json
import re
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
DIRS = [f'artifacts/validation/2026-11-04/{n}' for n in (
    'lidar_mount_adapted_v2', 'lidar_mount_pilot_v3', 'lidar_mount_speed_v4', 'lidar_packaging_v3',
    'lidar_jr_forward_v1', 'lidar_jr60_adoption_v1')]
TOKEN = re.compile(rb'(ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|sk-[A-Za-z0-9]{20,}|AKIA[0-9A-Z]{16}|'
                   rb'-----BEGIN [A-Z ]*PRIVATE KEY-----|xox[baprs]-[A-Za-z0-9-]{10,})')


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, check=True).stdout.decode('utf-8')


def main():
    untracked = git('ls-files', '--others', '--exclude-standard', '-z', '--', *DIRS).split('\0')
    ignored = git('ls-files', '--others', '--ignored', '--exclude-standard', '-z', '--', *DIRS).split('\0')
    rows = [(p, (ROOT/p).stat().st_size) for p in untracked if p and '__pycache__' not in p]
    by_dir = collections.defaultdict(lambda: [0, 0])
    by_ext = collections.defaultdict(lambda: [0, 0])
    flagged = dict(video=[], jsonl=[], over_1mb=[], pycache=[], token_like=[])
    for p, size in rows:
        d = '/'.join(p.split('/')[:4])
        by_dir[d][0] += 1; by_dir[d][1] += size
        ext = Path(p).suffix or Path(p).name
        by_ext[ext][0] += 1; by_ext[ext][1] += size
        if ext in ('.mp4', '.mkv', '.avi', '.webm'):
            flagged['video'].append([p, size])
        if ext == '.jsonl':
            flagged['jsonl'].append([p, size])
        if size > 1_000_000:
            flagged['over_1mb'].append([p, size])
        if size < 20_000_000:
            data = (ROOT/p).read_bytes()
            if TOKEN.search(data):
                flagged['token_like'].append(p)
    flagged['pycache'] = [p for p in untracked if '__pycache__' in p]
    result = dict(dirs=DIRS,
                  untracked_not_ignored=dict(count=len(rows), bytes=sum(s for _, s in rows)),
                  ignored_count=len([p for p in ignored if p]),
                  ignored_bytes=sum((ROOT/p).stat().st_size for p in ignored if p and (ROOT/p).is_file()),
                  by_dir={k: dict(count=v[0], bytes=v[1]) for k, v in sorted(by_dir.items())},
                  by_ext={k: dict(count=v[0], bytes=v[1]) for k, v in sorted(by_ext.items(), key=lambda kv: -kv[1][1])},
                  flagged=flagged)
    (HERE/'publication_scan.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n',
                                              encoding='utf-8', newline='\n')
    print(json.dumps({k: result[k] for k in ('untracked_not_ignored', 'ignored_count', 'ignored_bytes', 'by_dir')}, ensure_ascii=False))
    print(json.dumps({k: (v if k in ('token_like', 'pycache') else len(v)) for k, v in flagged.items()}, ensure_ascii=False)[:3000])


if __name__ == '__main__':
    main()
