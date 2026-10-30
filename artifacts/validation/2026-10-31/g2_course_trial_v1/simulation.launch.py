"""기존 공식 신호·30 cm 카메라 fixture에서 제어기 실행 파일만 바꾼다."""
import importlib.util
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
source=ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture/simulation.launch.py'
spec=importlib.util.spec_from_file_location('handoff_course_fixture',source)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
original_node=module.Node


def handoff_node(*args,**kwargs):
    if kwargs.get('package')=='arena_autonomy' and kwargs.get('executable')=='local_pursuit':
        kwargs['executable']='g1_g2_handoff_trial'
    return original_node(*args,**kwargs)


module.Node=handoff_node


def generate_launch_description():return module.generate_launch_description()
