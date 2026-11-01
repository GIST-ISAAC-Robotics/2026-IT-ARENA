"""3번 그리드에서 현행 2×5 신호와 옮긴 ID45를 쓰는 단독 프리랩. 시험 전용."""
import importlib.util
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
from prelap_signal_eval import DRIVER,FIXTURE,fixture_inputs

POLICIES = ('g1_short_boundary_prefix_enabled','g1_parallel_hold_trial','g1_pass_release_trial',
            'g1_guard_curve_trial','g1_traffic_recovery_trial','g1_traffic_path_recovery_trial',
            'g1_corridor_fallback_enabled')
VALUES = dict(g1_boundary_evidence_mode='corner_distance_candidate',
              g1_occupancy_timing_mode='per_ray_candidate',partial_observation_policy='bounded_local_v4')


def verify(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    output = Path(get('runtime_artifacts'))
    if (get('grid_slot')!='3' or get('actuation_mode')!='virtual_mcu' or get('lidar_acquisition')!='sequential'
            or get('lidar_compensation')!='both' or get('speed_bump')!='false' or get('chase_camera')!='true'
            or any(get(k)!='true' for k in POLICIES) or any(get(k)!=v for k,v in VALUES.items())
            or not output.is_absolute() or output/'follow_decisions.jsonl' != Path(get('g1_trace_path'))):
        raise ValueError('등록한 단독 경기 경로·신호·영상 시험 조건 불일치')
    fixture_inputs()
    return [ExecuteProcess(cmd=['/usr/bin/python3','-B',str(DRIVER/'signal_driver_v2.py'),str(output)],
                           output='screen')]


def generate_launch_description():
    spec = importlib.util.spec_from_file_location('prelap_current_simulation',
                ROOT/'src/arena_bringup/launch/simulation.launch.py')
    sim = importlib.util.module_from_spec(spec);spec.loader.exec_module(sim)
    # 현행 simulation launch의 센서·MCU·보호·카메라는 그대로 사용한다. 이 프로세스 안에서만
    # 공식 코스의 입력 디렉터리를 사전에 검토한 팀 파생본으로 바꾼다.
    sim.TRACK_DIRECTORIES = {**sim.TRACK_DIRECTORIES,'official':str(FIXTURE)}
    setup = [SetLaunchConfiguration(k,v) for k,v in dict(track='official',autonomy='true',
             traffic_light='false',autonomy_mode='local_pursuit',depth_camera='false',
             d435i_profile='low_load_30').items()]
    params = [SetParameter(name=k,value=True) for k in (*POLICIES,'g1_follow_enabled')]
    params += [SetParameter(name=k,value=v) for k,v in VALUES.items()]
    params += [SetParameter(name='g1_trace_path',value=LaunchConfiguration('g1_trace_path')),
               SetParameter(name='start_signal_mode',value='official_4red_local_candidate')]
    return LaunchDescription([*setup,*params,*sim.generate_launch_description().entities,OpaqueFunction(function=verify)])
