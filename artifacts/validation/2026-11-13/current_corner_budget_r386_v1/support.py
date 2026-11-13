"""격리한 r377 소스만 import한다. 현 작업 트리 공통 모듈을 섞지 않는다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
CHAIN=(
    'artifacts/validation/2026-11-13/released_current_r375_v1/candidate.py',
    'artifacts/validation/2026-11-13/current_corner_r370_v1/candidate.py',
    'artifacts/validation/2026-11-13/solo_crawl_retry_r352_v1/candidate.py',
    'artifacts/validation/2026-11-13/current_solo_gap_r346_v1/candidate.py',
    'artifacts/validation/2026-11-13/current_solo_low_r343_v1/candidate.py',
    'artifacts/validation/2026-11-12/current_region_policy_r281_v1/candidate.py',
    'artifacts/validation/2026-11-12/retained_route_execution_r266_v1/candidate_policy.py')


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def verify():
    manifest=json.loads((A/'source_manifest.json').read_bytes())
    changed=[name for name,h in manifest['files'].items() if not (ROOT/name).is_file() or digest(ROOT/name)!=h]
    assert not changed,('frozen source/input changed',changed)
    return manifest


def load(backend):
    for relative in reversed(('src/arena_autonomy','src/arena_vehicle_interface')):
        sys.path.insert(0,str(ROOT/relative))
    path=ROOT/'artifacts/validation/2026-11-13/released_current_r375_v1/compare.py'
    spec=importlib.util.spec_from_file_location('r386_frozen_compare',path)
    compare=importlib.util.module_from_spec(spec);spec.loader.exec_module(compare)
    compare.configure(backend)
    modules=[compare.policy,compare.policy.prior,compare.policy.prior.prior,compare.policy.prior.prior.prior,
        compare.policy.prior.prior.prior.prior,compare.policy.prior.prior.prior.prior.base,compare.policy.core_policy]
    found=[Path(m.__file__).resolve().relative_to(ROOT).as_posix() for m in modules]
    assert found==list(CHAIN)
    for name,module in list(sys.modules.items()):
        if name.startswith(('arena_autonomy','arena_vehicle_interface','candidate_recheck')) and getattr(module,'__file__',None):
            assert Path(module.__file__).resolve().is_relative_to(ROOT),('external module',name,module.__file__)
    native=None
    if backend=='native_v1':
        from arena_autonomy import _numeric_kernels
        assert _numeric_kernels.ABI_VERSION==1
        binary=Path(_numeric_kernels.__file__).resolve()
        assert binary.is_relative_to(ROOT/'src/arena_autonomy')
        native=dict(path=binary.relative_to(ROOT).as_posix(),sha256=digest(binary))
    return compare,dict(candidate_chain=found,native=native)
