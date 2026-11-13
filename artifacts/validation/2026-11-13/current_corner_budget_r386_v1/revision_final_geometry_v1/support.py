"""원 r386 자료/후보를 보존하고 선택 소스 arm만 import한다."""
import ast
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace

A=Path(__file__).resolve().parent
ROOT=Path(os.environ['R386_SOURCE_ROOT']).resolve()
INPUT=Path(os.environ['R386_INPUT_ROOT']).resolve()
CHAIN=(
    'artifacts/validation/2026-11-13/released_current_r375_v1/candidate.py',
    'artifacts/validation/2026-11-13/current_corner_r370_v1/candidate.py',
    'artifacts/validation/2026-11-13/solo_crawl_retry_r352_v1/candidate.py',
    'artifacts/validation/2026-11-13/current_solo_gap_r346_v1/candidate.py',
    'artifacts/validation/2026-11-13/current_solo_low_r343_v1/candidate.py',
    'artifacts/validation/2026-11-12/current_region_policy_r281_v1/candidate.py',
    'artifacts/validation/2026-11-12/retained_route_execution_r266_v1/candidate_policy.py')


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify():
    manifest=json.loads((A/'source_manifest.json').read_bytes());root=A.parent
    changed=[n for n,h in manifest['files'].items() if not (root/n).is_file() or digest(root/n)!=h]
    assert not changed,('source/input changed',changed)
    return manifest


def load(backend):
    for relative in reversed(('src/arena_autonomy','src/arena_vehicle_interface')):sys.path.insert(0,str(ROOT/relative))
    path=ROOT/'artifacts/validation/2026-11-13/released_current_r375_v1/compare.py'
    spec=importlib.util.spec_from_file_location('r386_selected_compare',path)
    compare=importlib.util.module_from_spec(spec);spec.loader.exec_module(compare);compare.configure(backend)
    modules=[compare.policy,compare.policy.prior,compare.policy.prior.prior,compare.policy.prior.prior.prior,
        compare.policy.prior.prior.prior.prior,compare.policy.prior.prior.prior.prior.base,compare.policy.core_policy]
    found=[Path(module.__file__).resolve().relative_to(ROOT).as_posix() for module in modules]
    assert found==list(CHAIN)
    for name,module in list(sys.modules.items()):
        if name.startswith(('arena_autonomy','arena_vehicle_interface','candidate_recheck')) and getattr(module,'__file__',None):
            assert Path(module.__file__).resolve().is_relative_to(ROOT),('external source',name,module.__file__)
    native=None
    if backend=='native_v1':
        from arena_autonomy import _numeric_kernels
        binary=Path(_numeric_kernels.__file__).resolve()
        assert _numeric_kernels.ABI_VERSION==1 and binary.is_relative_to(ROOT/'src/arena_autonomy')
        native=dict(path=binary.relative_to(ROOT).as_posix(),sha256=digest(binary))
    return compare,dict(candidate_chain=found,native=native)


def original_helpers():
    """원 compare의 입력목록/15반례/비용요약 함수 AST를 수정 없이 재사용한다."""
    original=INPUT/'artifacts/validation/2026-11-13/current_corner_budget_r386_v1/compare.py'
    tree=ast.parse(original.read_bytes());wanted=('percentile','summary','records','counterexample_records')
    selected=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in wanted]
    assert {n.name for n in selected}==set(wanted)
    scope=dict(A=original.parent,math=math,copy=copy,json=json)
    exec(compile(ast.Module(body=selected,type_ignores=[]),str(original),'exec'),scope)
    return SimpleNamespace(**{name:scope[name] for name in wanted})
