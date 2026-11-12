"""r304 전용 하위 launch 연결. 공용 launch 파일은 수정하지 않는다."""
import importlib.util
from pathlib import Path
A=Path(__file__).resolve().parent.parent;ROOT=A.parents[3]
spec=importlib.util.spec_from_file_location('r304_demo',ROOT/'src/arena_bringup/launch/demo.launch.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
original=module.get_package_share_directory
module.get_package_share_directory=lambda name:str(A) if name=='arena_bringup' else original(name)
generate_launch_description=module.generate_launch_description
