"""기존 2×5 신호와 동일 센서 차량. 300초 상대 각본 평가용 진입점."""
import importlib.util
from pathlib import Path
from launch.actions import ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

BASE=Path(__file__).resolve().parent
RECORDER=BASE.parents[1]/'2026-10-31/g1_continuous_prefix_course_v1/record_chase.py'


def recorder(context):
    get=lambda k:LaunchConfiguration(k).perform(context)
    run=Path(get('runtime_artifacts')); actor=Path(get('evaluation_actor_sdf'))
    if (get('evaluation_handoff_reserve') !='paced'
            or get('evaluation_handoff_rear_sweep')!='true' or get('evaluation_handoff_yield_progress')!='true' or get('speed_profile')!='g1_open_1p5' or actor!=run/'evaluation_actor.sdf'
            or not actor.is_file() or run.parent!=BASE):
        raise ValueError('등록된 300초 교통 시험 출력·차량 구성이 아님')
    return [ExecuteProcess(cmd=['/usr/bin/python3','-B',str(RECORDER),str(run)],output='screen')]


def generate_launch_description():
    parent=BASE.parent/'official_prelap_v1/course.launch.py'
    spec=importlib.util.spec_from_file_location('official_rgb_prelap',parent)
    entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
    description=entry.generate_launch_description()
    description.add_action(OpaqueFunction(function=recorder))
    return description
