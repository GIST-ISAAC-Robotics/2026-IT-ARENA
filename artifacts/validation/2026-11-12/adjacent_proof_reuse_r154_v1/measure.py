"""r150의 입력/기대 출력/계측을 그대로 사용하고 중복 검사만 제거한 후보를 비교한다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def main():
    sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    sources={str(p.relative_to(A)):sha(p) for p in (A/'candidate.py',Path(__file__))}
    # r150 모듈을 먼저 읽어 기존 패치 이전의 ORIGINAL 포인터를 보존한다.
    measure=load('r154_measure',A.parent/'adjacent_cost_r150_v1/measure.py')
    replacement=load('r154_candidate',A/'candidate.py')
    measure.candidate=replacement
    sys.argv=[str(Path(__file__)),'--label','pc_r154']
    measure.main()
    assert all(sha(A/name)==digest for name,digest in sources.items())
    with (A/'source_check.json').open('x') as f:json.dump(dict(files=sources,source_unchanged=True,
        result='../adjacent_cost_r150_v1/pc_r154_summary.json'),f,indent=2)


if __name__=='__main__':main()
