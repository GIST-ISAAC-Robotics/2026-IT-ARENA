"""기존 공식 신호·30 cm 카메라 fixture에서 제어기 실행 파일만 바꾼다."""
import types
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
source=ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture/simulation.launch.py'
text=source.read_text()
old_directory=source.parent.as_posix()
new_directory=(Path(__file__).resolve().parent/'fixture').as_posix()
assert text.count(repr(old_directory))==1
text=text.replace(repr(old_directory),repr(new_directory))
module=types.ModuleType('level_surface_fixture');module.__file__=str(source)
exec(compile(text,str(source),'exec'),module.__dict__)
original_node=module.Node


def handoff_node(*args,**kwargs):
    if kwargs.get('package')=='arena_autonomy' and kwargs.get('executable')=='local_pursuit':
        kwargs['executable']='g1_g2_handoff_trial'
    return original_node(*args,**kwargs)


module.Node=handoff_node


def generate_launch_description():return module.generate_launch_description()
