"""공통 소스를 바꾸지 않고 depth_support 내부 evaluate만 바꾼 비교 사본을 만든다."""
from pathlib import Path
import hashlib
import inspect
import json
from arena_autonomy import observation_support as original

A=Path(__file__).resolve().parent
source=inspect.getsource(original.depth_support)
start=source.index('    def evaluate(part, chosen):')
end=source.index('    # 광선/질의',start)
replacement='''    def evaluate(part, chosen):
        return evaluate_native(part, chosen, origins, directions, indices, valid, depth, pad, cycle_count)

'''
prefix='''"""r245 생성 비교 사본. 공통 패키지/기본값과 분리한다."""
import math
import numpy as np
from arena_autonomy.scan_topology import cyclic_count, adjacent_indices
from native_eval import evaluate_native

'''
target=A/'candidate.py'
assert not target.exists()
target.write_text(prefix+source[:start]+replacement+source[end:],encoding='utf8')
(A/'candidate_source.json').write_text(json.dumps(dict(
    source=original.__file__,source_sha256=hashlib.sha256(Path(original.__file__).read_bytes()).hexdigest(),
    candidate_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
    scope='only inner evaluate replaced; angle window, topology, padding and queries unchanged'),indent=2)+'\n')
