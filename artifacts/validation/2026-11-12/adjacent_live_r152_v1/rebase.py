"""기존 일반/평행 현재 재검사에 r142의 제한 GUARD 후보를 명시 연결한다."""
import importlib.util
from pathlib import Path

from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan

ROOT=Path(__file__).resolve().parents[4]


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


previous=load('r152_previous_rebase',ROOT/'artifacts/validation/2026-11-11/guard_recheck_r139_v1/rebase.py')
guard=load('r152_guard_rebase',ROOT/'artifacts/validation/2026-11-12/adjacent_current_r151_v1/current.py')


def guard_scope(output):
    if not guard.supported(output):return False
    normal=output[2]['normal'];sides=normal.get('selector_sides') or {}
    partial=normal.get('partial_observation') or {}
    return (set(sides)=={'left','right'} and all(s.get('present') is False for s in sides.values())
            and partial.get('allowed') is True and partial.get('reason')=='fully_observed')


def supported(output):
    return previous.supported(output) or guard_scope(output)


def rebase(output,submitted,current,scene,history,cfg):
    if previous.supported(output):
        return previous.rebase(output,submitted,current,scene,history,cfg)
    if not guard_scope(output):return None
    scan=MeasurementScan(**thaw(current['scan']['measurements']))
    return guard.rebase(output,submitted,current,scan,scene,history,cfg)
