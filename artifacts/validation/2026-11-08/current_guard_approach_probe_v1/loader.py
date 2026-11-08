import importlib.util
from pathlib import Path
import sys

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m)
    return m

def load():
    from arena_autonomy.local_follow import LocalFollow
    from arena_autonomy.pass_release_trial import PassReleaseSelector
    A=Path(__file__).resolve().parent
    base=module('detached_short_current',A.parent/'traffic_corner_prefix_probe_v1/candidate_local_follow.py')
    guard=module('detached_guard_approach',A/'candidate_selector.py')
    LocalFollow._baseline_command=base.LocalFollow._baseline_command
    PassReleaseSelector._guard_curve=guard.PassReleaseSelector._guard_curve
    return guard
