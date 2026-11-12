"""r149 판단과 r151 현재 재검사를 명시 선택하는 시험 연결."""
import importlib.util
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
path=ROOT/'artifacts/validation/2026-11-12/adjacent_mount_gap_r149_v1/candidate.py'
spec=importlib.util.spec_from_file_location('r152_original_planner',path)
planner=importlib.util.module_from_spec(spec);spec.loader.exec_module(planner)


def install():planner.install()


def worker_entry(*args):
    install()
    from arena_autonomy.decision_process import _worker
    return _worker(*args)
