"""저장 C1 원시 배치의 무반환/무효 표기를 집계한다. 점군은 공개 산출물에 쓰지 않는다."""
from collections import Counter
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
raw = ROOT/'build/r166b_recovered/artifacts/validation/2026-11-12/c1_ros_preview_r166_v1/capture/sdk_raw.jsonl'
vendor = ROOT/'build/c1_hardware_2026_11_10/remote/replay_root/recorded_replay/vendor_functions.inc'
counts = Counter()
scan_counts = []
flags = Counter()
for line in raw.open():
    data = json.loads(line)
    counter = Counter()
    for angle, distance, quality, flag in data['nodes']:
        key = ('positive_distance' if distance else 'zero_distance') + ('_positive_quality' if quality else '_zero_quality')
        counts[key] += 1; counter[key] += 1
        if not distance:
            flags[str(flag)] += 1
    scan_counts.append(counter)
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
result = dict(raw_sha256=sha(raw), vendor_excerpt_sha256=sha(vendor), scans=len(scan_counts),
              points=sum(counts.values()), counts=dict(counts), zero_distance_flags=dict(flags),
              scans_with_zero_distance=sum(any(k.startswith('zero_distance') and v for k,v in c.items()) for c in scan_counts),
              preview_zero_distance_encoding='JSON null -> NaN', vendor_zero_distance_encoding='+Inf',
              native_simulation_nonreturn_encoding='+Inf',
              scope='saved unmounted indoor capture; no physical no-return cause or timing validation; no live device use')
with (A/'summary.json').open('x', encoding='utf8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2); stream.write('\n')
print(json.dumps(result, ensure_ascii=False))
