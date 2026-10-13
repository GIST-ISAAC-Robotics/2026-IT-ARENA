"""자기 출력 로그 1개의 완료 후 해시 보충. 기존 manifest/zip은 수정하지 않는다."""
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
manifest = json.loads((BASE/'manifest.json').read_text())
log = BASE/'finalize_v2.log'
key = log.relative_to(ROOT).as_posix()
mismatches = [p for p, value in manifest['source_files_sha256'].items() if sha(ROOT/p) != value]
checks = {
    'prior_checks_true': manifest['passed'] and all(manifest['checks'].values()),
    'only_live_log_changed': mismatches == [key],
    'prior_log_hash_was_empty': manifest['source_files_sha256'].get(key) == hashlib.sha256(b'').hexdigest(),
    'completed_log_matches_checks': json.loads(log.read_text()) == manifest['checks'],
    'historical_zip_unchanged': sha(BASE/'final_source.zip') == manifest['final_source_zip_sha256'],
}
if not all(checks.values()):
    raise SystemExit(json.dumps(checks))
paths = [p for p in BASE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.name != 'seal.json']
with (BASE/'seal.json').open('x') as stream:
    json.dump({'passed': True, 'checks': checks, 'earlier_manifest_sha256': sha(BASE/'manifest.json'),
               'artifact_sha256': {p.relative_to(BASE).as_posix(): sha(p) for p in sorted(paths)},
               'limit': 'Supplement for completed log only; temporary solver executable still unavailable.'}, stream, indent=2)
print(json.dumps(checks, indent=2))
