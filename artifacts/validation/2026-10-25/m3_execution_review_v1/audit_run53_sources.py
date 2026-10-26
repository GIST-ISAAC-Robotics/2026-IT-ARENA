"""실행기의 보조 해시 실패와 평가기 자체 소스 보존을 분리한다. 결과 덮어쓰기 없음."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[4]
A=ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1'
D=A/'run53_queue_staggered'
digest=lambda raw:hashlib.sha256(raw).hexdigest()
report=json.loads((D/'run/report.json').read_text(encoding='utf8'))
checks={}
with zipfile.ZipFile(D/'run/source_snapshot.zip') as z:
    for path,expected in report['source_sha256'].items():
        checks[path]={'archive_matches_start':digest(z.read(path))==expected,
                      'current_matches_start':digest((ROOT/path).read_bytes())==expected}
copies={name:digest((A/'run52_queue_preflight'/saved).read_bytes())==digest((ROOT/name).read_bytes())
        for name,saved in [('scripts/m2_queue_eval.py','evaluator_source.py'),
                           ('tests/test_m2_queue_eval.py','test_source.py'),
                           ('config/tests/m2_queue_staggered_v1.json','overlay.json')]}
out={'runtime_source_count':len(checks),'runtime_source_unchanged_at_end':report['source_unchanged'],
     'all_runtime_archive_and_current_match':all(all(r.values()) for r in checks.values()),
     'runtime_files':checks, 'preflight_copies_still_match':copies,
     'wrapper_manifest_valid':False,
     'wrapper_failure':{'reason':'CRLF names passed to sha256sum with trailing CR; empty before/after manifests cannot prove invariance',
                        'carriage_returns':(D/'files_to_hash.txt').read_bytes().count(b'\r'),
                        'before_bytes':(D/'source_sha256_before.txt').stat().st_size,
                        'after_bytes':(D/'source_sha256_after.txt').stat().st_size},
     'scope':'runtime archive/start/end/current proof is separate from failed wrapper manifest; no rerun or original verdict edits'}
path=D/'source_archive_audit.json'
assert not path.exists()
path.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in out.items() if k!='runtime_files'},ensure_ascii=False))
