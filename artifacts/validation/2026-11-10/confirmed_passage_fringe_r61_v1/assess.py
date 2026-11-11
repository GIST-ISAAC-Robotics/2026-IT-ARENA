"""기존 코너 병렬 평가기를 해시 확인 뒤 같은 기준으로 실행한다."""
import hashlib
import importlib.util
import json
from pathlib import Path

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
lock=json.loads((A/'registered.json').read_bytes())['original_evaluator']
assert len(lock)==1
path,digest=next(iter(lock.items()))
assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==digest
spec=importlib.util.spec_from_file_location('corner_parallel_original_evaluator',ROOT/path)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.HERE=A.parent
raise SystemExit(module.main(A.name))
