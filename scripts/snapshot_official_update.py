#!/usr/bin/env python3
"""공식 저장소의 고정 커밋·릴리스·이슈 원문을 읽기 전용으로 보존합니다."""
import hashlib
import json
from pathlib import Path
import subprocess
import urllib.parse
import urllib.request
from datetime import datetime, timezone

REPO = Path(__file__).resolve().parents[1]
REMOTE = 'MOSW626/istech-it-arena'


def api(path):
    return subprocess.check_output(['gh', 'api', f'repos/{REMOTE}/{path}'])


def main():
    out = REPO / 'artifacts/validation/2026-09-15/official_update/upstream'
    out.mkdir(parents=True, exist_ok=False)
    head = json.loads(api('commits/main'))
    release = json.loads(api('releases/latest'))
    sha = head['sha']
    records = {'commit.json': head, 'release.json': release,
               'tree.json': json.loads(api(f'git/trees/{sha}?recursive=1'))}
    records['issues.json'] = json.loads(api('issues?state=all&per_page=100'))
    for issue in records['issues.json']:
        n = issue['number']
        records[f'issue_{n}_comments.json'] = json.loads(api(f'issues/{n}/comments?per_page=100'))
    for name, obj in records.items():
        (out / name).write_text(json.dumps(obj, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    files = ['MANUAL.md', 'README.md', 'track/README.md', 'meetings/2026-09-14 미팅.md']
    for name in files:
        data = subprocess.check_output(['gh', 'api', f'repos/{REMOTE}/contents/{urllib.parse.quote(name)}?ref={sha}',
                                       '-H', 'Accept: application/vnd.github.raw+json'])
        target = out / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    asset = release['assets'][0]
    destination = REPO / 'assets/track/official' / release['tag_name']
    destination.mkdir(parents=True, exist_ok=False)
    request = urllib.request.Request(asset['browser_download_url'], headers={'User-Agent': 'IT-ARENA-audit'})
    with urllib.request.urlopen(request, timeout=60) as response:
        data = response.read()
    (destination / asset['name']).write_bytes(data)
    manifest = {'observed_at_utc': datetime.now(timezone.utc).isoformat(), 'commit': sha,
                'release': release['tag_name'], 'asset': asset['name'], 'url': asset['browser_download_url'],
                'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
