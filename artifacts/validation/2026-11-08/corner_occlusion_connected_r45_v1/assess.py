"""r30 기준 평가기의 바이트를 확인하고 새 출력 폴더만 대입한다."""
from pathlib import Path
import hashlib
import importlib.util
import json

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
lock = json.loads((A/'evaluation_source.json').read_bytes())
assert len(lock) == 1
path, digest = next(iter(lock.items()))
assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest() == digest
spec = importlib.util.spec_from_file_location('original_r30_corner_assess', ROOT/path)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
m.HERE = A.parent
raise SystemExit(m.main(A.name))
