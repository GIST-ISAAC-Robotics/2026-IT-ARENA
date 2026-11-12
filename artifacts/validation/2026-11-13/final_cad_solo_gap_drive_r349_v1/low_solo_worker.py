"""spawn 자식에서 선택형 현재 검사만 r346으로 대체한다."""
import importlib.util
from pathlib import Path

A = Path(__file__).resolve().parent


def worker(stage, *args):
    import worker_candidate as prior
    import route_proof
    old = route_proof.rebase
    boundary = prior.load_candidate()
    if stage == 'plan':
        boundary.install()
    elif stage == 'current':
        path = A.parent/'current_solo_gap_r346_v1/candidate.py'
        spec = importlib.util.spec_from_file_location('r346_current_worker', path)
        policy = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(policy)
        route_proof.rebase = policy.rebase
    try:
        prior.ORIGINAL(stage, *args)
    finally:
        if stage == 'plan':
            boundary.uninstall()
        route_proof.rebase = old


def install():
    import overlap
    overlap._worker = worker
