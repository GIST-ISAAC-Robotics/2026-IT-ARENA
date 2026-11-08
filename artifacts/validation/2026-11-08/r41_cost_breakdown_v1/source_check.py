"""Pass B가 기록한 실제 로드 모듈 해시를 r42 v1/v3 manifest와 대조한다(읽기 전용)."""
import json
from pathlib import Path
HERE = Path(__file__).resolve().parent
REF = HERE.parent/'jetson_bundle_refresh_v1'
mods = json.loads((HERE/'profile_result.json').read_text(encoding='utf-8'))['source_modules']
out = {}
for label, path in (('v1', REF/'bundle_manifest.json'), ('v3', REF/'v3/bundle_manifest.json')):
    files = json.loads(path.read_text(encoding='utf-8'))['files']
    present = {m['path']: files[m['path']]['sha256'] == m['sha256'] for m in mods.values() if m['path'] in files}
    out[label] = dict(loaded=len(mods), in_manifest=len(present), identical=sum(present.values()),
                      different=sorted(k for k, v in present.items() if not v),
                      not_in_manifest=sorted(m['path'] for m in mods.values() if m['path'] not in files))
(HERE/'source_check.json').write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(out, ensure_ascii=False))

# 두 실행 이후(현재) 디스크 해시도 v3 manifest와 대조한다. Pass C는 Pass B 직후 같은 파일을 읽었다.
from hashlib import sha256
REPO = HERE.parents[3]
files = json.loads((REF/'v3/bundle_manifest.json').read_text(encoding='utf-8'))['files']
now = {m['path']: sha256((REPO/m['path']).read_bytes()).hexdigest() == files[m['path']]['sha256'] for m in mods.values()}
out['v3_after_runs_disk'] = dict(checked=len(now), identical=sum(now.values()))
(HERE/'source_check.json').write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding='utf-8')
print(out['v3_after_runs_disk'])
