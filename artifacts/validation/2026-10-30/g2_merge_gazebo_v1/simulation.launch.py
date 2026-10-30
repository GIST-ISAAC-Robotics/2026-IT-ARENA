"""기존 중앙 표적 fixture의 센서 오차와 명시 자동 전환 실행 파일을 선택한다."""
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


original_node = module.Node

def handoff_node(*args,**kwargs):
    if kwargs.get('package')=='arena_autonomy' and kwargs.get('executable')=='g2_entry_trial':
        kwargs['executable']='g1_g2_handoff_trial'
    return original_node(*args,**kwargs)

module.Node=handoff_node
