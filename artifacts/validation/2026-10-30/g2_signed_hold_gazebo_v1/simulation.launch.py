"""기존 중앙 표적 fixture에 센서 모델 오차만 적용한다."""
import importlib.util
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))
from sensor_stress import apply_stress

source = BASE.parent/'g2_center_gazebo_v1/fixture/simulation.launch.py'
spec = importlib.util.spec_from_file_location('g2_stress_fixture', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
original = module._b0183_model


def stressed_model(model_xml, profile):
    return apply_stress(original(model_xml, profile))


module._b0183_model = stressed_model


def generate_launch_description():
    return module.generate_launch_description()
