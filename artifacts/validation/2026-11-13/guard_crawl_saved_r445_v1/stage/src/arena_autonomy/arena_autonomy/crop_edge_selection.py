"""기본 OFF인 경계 성분 후보의 공통 설정 검사. 입력/시각/허가는 변환하지 않는다."""


def validate_policy_selection(enabled, policy, traffic_recovery):
    if type(enabled) is not bool:
        raise ValueError('crop edge component trial must be bool')
    if enabled and (policy != 'bounded_local_v4' or traffic_recovery is not True):
        raise ValueError('crop edge component requires bounded_local_v4 and traffic recovery')


def validate_node_selection(node, enabled):
    if not enabled:
        return
    from arena_autonomy.native_lidar_input import input_representation
    motion = node.motion
    if (node.get_parameter('use_sim_time').value is not True
            or node.hardware_inputs or motion is None or motion.mode != 'both'
            or motion.verified is not True or motion.timing_policy != 'verified_only'
            or input_representation(node) != 'native_simulation_v1'):
        raise ValueError('crop edge component requires verified-only native simulation with both motion')
