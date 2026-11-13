"""r460 평가 사본: raw XML 변형 → 공통 override → r457 시설/RTF 처리."""
import importlib.util
import os
from pathlib import Path
import sys

A = Path(__file__).resolve().parent.parent
ROOT = A.parents[3]
sys.path.insert(0, str(A))
from prepare import CASES
from variation_launch import runtime_wrapper, spawn_clone

name = os.environ['IT_ARENA_R460_VARIATION']
assert name in CASES
evidence = A/'cases'/name
spec = importlib.util.spec_from_file_location('r460_simulation', ROOT/'src/arena_bringup/launch/simulation.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module._runtime_world_overrides = runtime_wrapper(module._runtime_world_overrides, name, evidence)
module._launch_setup = spawn_clone(module._launch_setup, name, evidence)
generate_launch_description = module.generate_launch_description
