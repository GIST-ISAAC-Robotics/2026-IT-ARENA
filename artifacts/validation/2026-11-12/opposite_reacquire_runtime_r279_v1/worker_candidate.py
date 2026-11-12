"""명시 r279 시험의 spawned 계획 프로세스에만 r278 후보를 설치한다."""
import importlib.util
from pathlib import Path
import sys

A=Path(__file__).resolve().parent
sys.path.insert(0,str(A.parent/'retained_route_execution_r266_v1'))
import overlap
ORIGINAL=overlap._worker


def load_candidate():
    path=A.parent/'terminal_boundary_r278_v1/candidate.py'
    spec=importlib.util.spec_from_file_location('r278_boundary_candidate',path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def worker(stage,*args):
    candidate=load_candidate()
    if stage=='plan':candidate.install()
    try:ORIGINAL(stage,*args)
    finally:
        if stage=='plan':candidate.uninstall()


def install():overlap._worker=worker
