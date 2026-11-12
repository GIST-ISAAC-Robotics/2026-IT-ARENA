"""현재 소스·비교 도구·불변 저장 입력만 격리 묶음으로 보존한다."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[4]
A = Path(__file__).resolve().parent
parent = ROOT/'artifacts/validation/2026-11-12/combined_ros_cost_r232_v1'
original = json.loads((parent/'bundle_manifest.json').read_bytes())
publisher = (parent/'publisher_source.py').read_text(encoding='utf-8').replace(
    "TOPIC='/arena_probe_r232/imu'", "TOPIC='/arena_probe_r235/imu'")
(A/'publisher_source.py').write_text(publisher, encoding='utf-8')
paths = {name for name in original['files'] if name.startswith('src/')}
paths.update(str(path.relative_to(ROOT)).replace('\\','/') for path in A.glob('*.py'))
files = {name: dict(size=(ROOT/name).stat().st_size, sha256=hashlib.sha256((ROOT/name).read_bytes()).hexdigest())
         for name in sorted(paths)}
trace = original['trace']
assert hashlib.sha256((ROOT/trace).read_bytes()).hexdigest() == original['trace_sha256']
manifest = dict(trace=trace, trace_sha256=original['trace_sha256'],
                parent_manifest_sha256=hashlib.sha256((parent/'bundle_manifest.json').read_bytes()).hexdigest(), files=files)
(A/'bundle_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
target = ROOT/'build/r235_bundle.zip'
with zipfile.ZipFile(target, 'w', zipfile.ZIP_DEFLATED) as bundle:
    for name in sorted(paths | {trace, str((A/'bundle_manifest.json').relative_to(ROOT)).replace('\\','/')}):
        bundle.write(ROOT/name, name)
result = dict(path='build/r235_bundle.zip', bytes=target.stat().st_size,
              sha256=hashlib.sha256(target.read_bytes()).hexdigest(), files=len(files), raw_input_git_excluded=True)
(A/'bundle_receipt.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
print(json.dumps(result))
