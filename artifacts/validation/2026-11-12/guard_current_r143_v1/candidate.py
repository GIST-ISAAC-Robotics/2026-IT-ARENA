"""r139 판단 후보를 그대로 실행한다. 새 변경은 부모의 현재 재검사에만 있다."""
import importlib.util
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
path=ROOT/'artifacts/validation/2026-11-11/guard_recheck_r139_v1/candidate.py'
spec=importlib.util.spec_from_file_location('r143_original_planner',path)
planner=importlib.util.module_from_spec(spec);spec.loader.exec_module(planner)


def install():planner.install()


def worker_entry(*args):
    install()
    from arena_autonomy.decision_process import _worker
    return _worker(*args)
