"""실물 추정 시각의 명시 선택. MCU ARM이나 물리 출력 권한은 부여하지 않는다."""

HARDWARE_ENVIRONMENTS = ('hardware_shadow_v1', 'hardware_runtime_candidate_v1')


def configure(node):
    environment = node.declare_parameter('execution_environment', 'default').value
    if environment not in ('default', *HARDWARE_ENVIRONMENTS):
        raise ValueError('unknown execution environment')
    if environment in HARDWARE_ENVIRONMENTS and node.get_parameter('use_sim_time').value:
        raise ValueError('hardware execution requires the real ROS clock')
    node.execution_environment = environment
    node.hardware_shadow = environment == 'hardware_shadow_v1'
    node.hardware_runtime = environment == 'hardware_runtime_candidate_v1'
    node.hardware_inputs = environment in HARDWARE_ENVIRONMENTS
    return environment


def estimated_hardware(node):
    return getattr(node, 'execution_environment', None) in HARDWARE_ENVIRONMENTS


def validate_motion_time(node, mode, verified, policy):
    if policy not in ('verified_only', 'host_estimated_v1'):
        raise ValueError('unknown motion scan time policy')
    if policy == 'host_estimated_v1' and (
            not estimated_hardware(node) or node.get_parameter('use_sim_time').value
            or verified or mode != 'both'):
        raise ValueError('host estimate policy requires explicit physical-clock hardware execution, '
                         'unverified timing and both compensation')
    if getattr(node, 'execution_environment', None) == 'hardware_runtime_candidate_v1' and (
            policy != 'host_estimated_v1' or verified or mode != 'both'):
        raise ValueError('hardware runtime candidate must preserve the estimated acquisition basis')


def motion_supported(node):
    motion = node.motion
    return motion is not None and motion.mode == 'both' and (
        motion.verified or (estimated_hardware(node)
                            and motion.timing_policy == 'host_estimated_v1'
                            and not node.get_parameter('use_sim_time').value))
