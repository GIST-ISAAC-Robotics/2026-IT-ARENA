"""기존 자율주행 재생 기록과 출력 없는 실물 입력 점검 기록을 분리한다."""
from arena_vehicle_interface.bag_contract import inputs_for, required_for, topics_for

PROBE_SCHEMA = 'arena_sensor_probe_v1'


def select_profile(name, clock_mode, sensor_topics, feedback_mode):
    if name == 'autonomy_sim':
        if clock_mode not in (None, 'simulation') or sensor_topics:
            raise ValueError('autonomy_sim requires simulation clock and its unchanged topic contract')
        return dict(name=name, schema=2, clock_mode='simulation', use_sim_time=True,
                    topics=topics_for(feedback_mode), required=required_for(feedback_mode),
                    parameter_nodes=('local_pursuit', 'lidar_safety'))
    if name != 'sensor_probe' or clock_mode not in ('simulation', 'system'):
        raise ValueError('sensor_probe requires an explicit simulation or system clock mode')
    if (type(sensor_topics) not in (list, tuple) or not sensor_topics
            or any(type(t) is not str for t in sensor_topics)
            or len(set(sensor_topics)) != len(sensor_topics)):
        raise ValueError('explicit unique sensor topics required')
    allowed = inputs_for(feedback_mode)
    if any(t not in allowed for t in sensor_topics):
        raise ValueError('sensor_probe accepts allowlisted sensors only, not commands/truth')
    return dict(name=name, schema=PROBE_SCHEMA, clock_mode=clock_mode,
                use_sim_time=clock_mode == 'simulation',
                topics={t: allowed[t] for t in sensor_topics}, required=set(sensor_topics),
                parameter_nodes=())


def profile_metadata(profile):
    return dict(recording_profile=profile['name'], recorder_clock_mode=profile['clock_mode'],
                recorder_use_sim_time=profile['use_sim_time'],
                sensor_acquisition_time_verified=False,
                controller_replay_eligible=profile['name'] == 'autonomy_sim')


def capture_complete(profile, errors, dropped, regressions, parameters, stats):
    return (not errors and not dropped and not regressions
            and set(parameters) == set(profile['parameter_nodes'])
            and all(stats.get(t, {}).get('count', 0) > 0 for t in profile['required']))
