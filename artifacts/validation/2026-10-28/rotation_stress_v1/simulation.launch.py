"""공식 fixture의 센서 모델에만 시험 오차를 더한다."""
import importlib.util
from pathlib import Path
import sys

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path.insert(0,str(BASE))
from sensor_stress import apply_stress

source=ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture/simulation.launch.py'
spec=importlib.util.spec_from_file_location('rotation_stress_base_simulation',source)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
original=module._b0183_model


def stressed_model(model_xml,profile):
    return apply_stress(original(model_xml,profile))


module._b0183_model=stressed_model


def generate_launch_description():
    return module.generate_launch_description()
