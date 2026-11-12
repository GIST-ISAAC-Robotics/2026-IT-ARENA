"""명시 r282 spawned 자식 안에만 계획/현재 검사 후보를 설치한다."""
import importlib.util
from pathlib import Path
import sys
A=Path(__file__).resolve().parent
sys.path.insert(0,str(A.parent/'retained_route_execution_r266_v1'))
import overlap
ORIGINAL=overlap._worker


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def load_candidate():
    return load('r278_boundary_candidate',A.parent/'terminal_boundary_r278_v1/candidate.py')


def load_current_candidate():
    return load('r281_current_candidate',A.parent/'current_region_policy_r281_v1/candidate.py')


def worker(stage,*args):
    import route_proof
    original=route_proof.rebase
    boundary=load_candidate()
    if stage=='plan':boundary.install()
    elif stage=='current':route_proof.rebase=load_current_candidate().rebase
    try:ORIGINAL(stage,*args)
    finally:
        if stage=='plan':boundary.uninstall()
        route_proof.rebase=original


def install():overlap._worker=worker
