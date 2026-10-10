"""G1-a 선택 실행. 기존 단독 모드·독립 보호층·신호 허가를 유지한다."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter


def generate_launch_description():
    # 새 bool은 LocalPursuit만 소비한다. 구동/센서/보호 매개변수는 덮어쓰지 않는다.
    launch_file = Path(get_package_share_directory('arena_bringup'))/'launch/local_pursuit.launch.py'
    # 종료 시 TimerAction도 하위 launch의 timeout 설정을 읽어야 한다.
    # 독립 진입점이므로 문맥을 조기에 pop하는 scoped group은 사용하지 않는다.
    return LaunchDescription([GroupAction(scoped=False, actions=[
        SetParameter(name='g1_follow_enabled', value=True),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file))),
    ])])
