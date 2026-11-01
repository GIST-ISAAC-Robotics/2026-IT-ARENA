"""기존 2×5 출발 시설과 기록기를 명시 300초 시험에만 연결한다."""
import importlib.util
from pathlib import Path
from launch.actions import ExecuteProcess,OpaqueFunction
from launch.substitutions import LaunchConfiguration

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
RECORDER=ROOT/'artifacts/validation/2026-10-31/g1_continuous_prefix_course_v1/record_chase.py'


def recorder(context):
    value=lambda k:LaunchConfiguration(k).perform(context)
    output=Path(value('runtime_artifacts'));actor=Path(value('evaluation_actor_sdf'))
    if (output!=HERE/'run1' or actor!=output/'evaluation_actor.sdf' or not actor.is_file()
            or value('evaluation_handoff_reserve')!='recent' or value('speed_profile')!='g1_open_1p5'):
        raise ValueError('등록된 경기 거리·최근 전진 근거 시험만 허용')
    return [ExecuteProcess(cmd=['/usr/bin/python3','-B',str(RECORDER),str(output)],output='screen')]


def generate_launch_description():
    file=HERE.parent/'official_prelap_v1/course.launch.py'
    spec=importlib.util.spec_from_file_location('five_lap_2x5_fixture',file)
    parent=importlib.util.module_from_spec(spec);spec.loader.exec_module(parent)
    result=parent.generate_launch_description();result.add_action(OpaqueFunction(function=recorder))
    return result
