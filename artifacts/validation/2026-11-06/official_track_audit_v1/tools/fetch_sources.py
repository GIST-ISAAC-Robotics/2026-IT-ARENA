"""공식 v2026.11.06 원문·릴리스 자산을 고정 커밋으로 보존한다.

- main/태그/릴리스 메타데이터와 이전 감사 기준(17178293) 이후 compare(패치 본문 제외)를 저장한다.
- 문서·생성 코드 원문을 고정 커밋에서 받아 upstream/ 아래에 둔다.
- 릴리스 ZIP을 assets/track/official/v2026.11.06/에 받고 GitHub digest와 SHA-256을 대조한다.
- 같은 경로의 다른 바이트는 덮어쓰지 않는다. 실행 월드·공용 코드는 건드리지 않는다.
"""
from pathlib import Path
import base64
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from urllib.parse import quote
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[5]
ART = Path(__file__).resolve().parents[1]
OUT = ART / 'upstream'
REMOTE = 'MOSW626/istech-it-arena'
EXPECT_MAIN = '45691aec825ebfe4ff3edd9f221118e0c5026d1a'
PREV_MAIN = '17178293029d446fa603f3229556dd865bbddd38'
TAG = 'v2026.11.06'
ASSET_DIR = ROOT / 'assets/track/official' / TAG


def api(path, raw=False):
    if raw:
        # Windows gh raw 출력 변환을 피하려고 contents API base64를 복원한다.
        record = api(path)
        if record.get('encoding') == 'base64' and record.get('content'):
            return base64.b64decode(record['content'])
        # 1 MB 초과 파일은 blob API로 받는다.
        blob = api('git/blobs/' + record['sha'])
        return base64.b64decode(blob['content'])
    data = subprocess.check_output(['gh', 'api', f'repos/{REMOTE}/{path}'])
    return json.loads(data)


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if path.read_bytes() != data:
            raise RuntimeError(f'refuse overwrite: {path}')
    else:
        path.write_bytes(data)


def dump(obj):
    return (json.dumps(obj, ensure_ascii=False, indent=2) + '\n').encode('utf-8')


def git_blob_sha1(data):
    return hashlib.sha1(b'blob %d\0' % len(data) + data).hexdigest()


def main():
    fetched_at = datetime.now(timezone.utc).isoformat()
    head = api('commits/main')
    if head['sha'] != EXPECT_MAIN:
        print('main moved:', head['sha'], file=sys.stderr)
    sha = head['sha']
    release = api('releases/latest')
    tag_ref = api('git/ref/tags/' + TAG)
    tag_commit = api('commits/' + TAG)
    compare = api(f'compare/{PREV_MAIN}...{sha}')
    for f in compare.get('files', []):
        f.pop('patch', None)
    tree = api(f'git/trees/{sha}?recursive=1')
    records = {
        'commit_main.json': head, 'release_latest.json': release,
        'tag_ref.json': tag_ref, 'tag_commit.json': tag_commit,
        'compare_since_17178293_nopatch.json': compare, 'tree_main.json': tree,
    }
    for name, obj in records.items():
        save(OUT / 'api' / name, dump(obj))
    paths = ['README.md', 'MANUAL.md', 'track/README.md', 'track/design_final.json',
             'track/make_design_610.py', 'track/track_gen.py', 'docs/venue/README.md',
             'docs/venue/make_save_points.py', 'docs/venue/save_points.png']
    blob_by_path = {e['path']: e['sha'] for e in tree['tree'] if e['type'] == 'blob'}
    texts = []
    for name in paths:
        data = api('contents/' + quote(name) + '?ref=' + sha, raw=True)
        if git_blob_sha1(data) != blob_by_path[name]:
            raise RuntimeError(f'blob mismatch {name}')
        save(OUT / 'main' / name, data)
        texts.append({'path': name, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
                      'git_blob': blob_by_path[name]})
    assets = []
    for item in release['assets']:
        target = ASSET_DIR / item['name']
        req = Request(item['browser_download_url'], headers={'User-Agent': 'IT-ARENA-source-audit'})
        with urlopen(req, timeout=120) as response:
            data = response.read()
        save(target, data)
        digest = hashlib.sha256(data).hexdigest()
        gh_digest = item.get('digest')
        assets.append({'name': item['name'], 'url': item['browser_download_url'], 'bytes': len(data),
                       'api_size': item['size'], 'sha256': digest, 'github_digest': gh_digest,
                       'digest_match': gh_digest == 'sha256:' + digest,
                       'size_match': len(data) == item['size'],
                       'updated_at': item.get('updated_at')})
    manifest = {'fetched_at_utc': fetched_at, 'remote': REMOTE, 'main_sha': sha,
                'main_expected': EXPECT_MAIN, 'tag': TAG, 'tag_ref_object': tag_ref['object'],
                'tag_commit_sha': tag_commit['sha'], 'release_published_at': release['published_at'],
                'release_tag_name': release['tag_name'], 'release_target_commitish': release['target_commitish'],
                'previous_audit_main': PREV_MAIN, 'compare_total_commits': compare['total_commits'],
                'compare_files': [{'filename': f['filename'], 'status': f['status'], 'sha': f.get('sha')}
                                  for f in compare.get('files', [])],
                'upstream_texts': texts, 'release_assets': assets}
    save(OUT / 'source_manifest.json', dump(manifest))
    (ASSET_DIR / 'SHA256SUMS.txt').exists() or save(
        ASSET_DIR / 'SHA256SUMS.txt',
        ''.join(f"{a['sha256']}  {a['name']}\n" for a in assets).encode())
    print(json.dumps({'main': sha, 'tag_commit': tag_commit['sha'], 'assets': assets}, indent=2))


if __name__ == '__main__':
    main()
