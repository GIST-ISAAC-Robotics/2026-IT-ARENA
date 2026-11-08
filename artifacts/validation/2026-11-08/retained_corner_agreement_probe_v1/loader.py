import importlib.util
from pathlib import Path
import sys
def load():
    spec=importlib.util.spec_from_file_location('detached_retained_corner',Path(__file__).with_name('candidate_local_follow.py'))
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    return module
