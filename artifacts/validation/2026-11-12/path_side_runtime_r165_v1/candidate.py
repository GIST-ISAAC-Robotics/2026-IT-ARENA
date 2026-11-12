"""r164를 명시한 단일 시험 프로세스/계산 자식 연결."""
import importlib.util
from pathlib import Path
p=Path(__file__).resolve().parent.parent/'path_side_gap_r164_v1/candidate.py'
spec=importlib.util.spec_from_file_location('r165_planner',p)
planner=importlib.util.module_from_spec(spec);spec.loader.exec_module(planner)
def install():planner.install()
def worker_entry(*args):
 install()
 from arena_autonomy.decision_process import _worker
 return _worker(*args)
