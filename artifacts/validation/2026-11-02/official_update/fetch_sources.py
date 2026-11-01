"""공식 원문을 고정 커밋/릴리스로 보존한다. 실행 월드를 교체하지 않는다."""
from pathlib import Path
import hashlib
import json
import subprocess
import base64
from datetime import datetime, timezone
from urllib.parse import quote
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent / 'upstream'
REMOTE = 'MOSW626/istech-it-arena'


def api(path, raw=False):
    # Windows gh의 raw PNG 출력 변환 오류를 피하고 API base64 바이트를 복원한다.
    if raw and path.split('?')[0].endswith('.png'):
        record = api(path)
        return base64.b64decode(record['content'])
    args = ['gh', 'api', f'repos/{REMOTE}/{path}']
    if raw:
        args += ['-H', 'Accept: application/vnd.github.raw+json']
    data = subprocess.check_output(args)
    return data if raw else json.loads(data)


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if path.read_bytes() != data:
            raise RuntimeError(f'refuse overwrite: {path}')
    else:
        path.write_bytes(data)


def main():
    head = api('commits/main')
    release = api('releases/latest')
    sha = head['sha']
    release_commit = api('commits/' + release['tag_name'])
    records = {'commit.json': head, 'release.json': release,
               'release_commit.json': release_commit,
               'compare_since_1024.json': api('compare/d8107de6ccff14ae17db7fb08732b55fed82b5b4...' + sha)}
    for n in (2, 3, 6, 16):
        records[f'issue_{n}.json'] = api(f'issues/{n}')
        records[f'issue_{n}_comments.json'] = api(f'issues/{n}/comments?per_page=100')
    for name, data in records.items():
        save(OUT / name, (json.dumps(data, ensure_ascii=False, indent=2)+'\n').encode())
    paths = ['README.md', 'MANUAL.md', 'track/README.md', 'track/track_gen.py',
             'track/output_final/scene.json', 'track/output_final/world.sdf',
             'track/output_final/traffic_light.py', 'meetings/2026-11-01 미팅.md',
             'meetings/노션 원문/2026-11-01 미팅 (노션 원문).md',
             'docs/venue/save_points.png', 'track/design_final.json', 'parts/speed_bump/README.md']
    for name in paths:
        save(OUT / name, api('contents/' + quote(name) + '?ref=' + sha, raw=True))
    assets = []
    for item in release['assets']:
        target = ROOT / 'assets/track/official' / release['tag_name'] / item['name']
        with urlopen(Request(item['browser_download_url'], headers={'User-Agent':'IT-ARENA-source-audit'}), timeout=60) as response:
            data = response.read()
        save(target, data)
        assets.append(dict(path=target.relative_to(ROOT).as_posix(), url=item['browser_download_url'],
                           bytes=len(data), sha256=hashlib.sha256(data).hexdigest()))
    files = [dict(path=p.relative_to(ROOT).as_posix(), bytes=p.stat().st_size,
                  sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sorted(OUT.rglob('*')) if p.is_file()]
    result = dict(observed_at_utc=datetime.now(timezone.utc).isoformat(), main_sha=sha,
                  release=release['tag_name'], release_sha=release_commit['sha'], assets=assets, files=files,
                  runtime_world_updated=False)
    (OUT.parent / 'source_manifest.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='files'}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
