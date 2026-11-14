"""기본 OFF인 경계 성분 후보의 공통 설정 검사. 입력/시각/허가는 변환하지 않는다."""

SIMULATION_SCOPE = 'verified_native_simulation_v1'
ESTIMATED_SCOPE = 'host_estimated_crop_v1'


def validate_observation_scope(scope):
    if type(scope) is not str or scope not in (SIMULATION_SCOPE, ESTIMATED_SCOPE):
        raise ValueError('explicit registered observation input scope required')
    return scope


def estimated_environment(*, execution_environment, use_sim_time, motion_verified,
                          motion_time_policy, motion_mode, lidar_input_representation):
    return (execution_environment == 'hardware_runtime_candidate_v1' and use_sim_time is False
            and motion_verified is False and motion_time_policy == 'host_estimated_v1'
            and motion_mode == 'both' and lidar_input_representation == 'native_estimated_v1')


def basis_matches(scope, policy, basis):
    validate_observation_scope(scope)
    return ((policy, basis) == (('host_estimated_v1', 'unverified_estimate')
            if scope == ESTIMATED_SCOPE else ('verified_only', 'simulation_acquisition')))


def timing_matches(scope, verified, policy, basis):
    return (verified is (False if scope == ESTIMATED_SCOPE else True)
            and basis_matches(scope, policy, basis))


def component_scope(scope):
    validate_observation_scope(scope)
    return ('bounded_estimated_edge_component_not_free_space' if scope == ESTIMATED_SCOPE
            else 'bounded_simulation_edge_component_not_free_space')


def configure_follow_observation(follow, scope):
    """추정 선택만 인스턴스에 연결한다. 기본 FollowConfig/직렬화는 바꾸지 않는다."""
    validate_observation_scope(scope)
    if scope == ESTIMATED_SCOPE:
        normal = getattr(follow, 'normal', follow)
        if getattr(normal, 'partial_policy', None) is None:
            raise ValueError('estimated observation requires a bounded partial observation policy')
        normal.partial_policy.observation_input_scope = scope


def validate_policy_selection(enabled, policy, traffic_recovery):
    if type(enabled) is not bool:
        raise ValueError('crop edge component trial must be bool')
    if enabled and (policy != 'bounded_local_v4' or traffic_recovery is not True):
        raise ValueError('crop edge component requires bounded_local_v4 and traffic recovery')


def validate_node_selection(node, enabled, *, observation_scope=SIMULATION_SCOPE):
    validate_observation_scope(observation_scope)
    if not enabled and observation_scope == SIMULATION_SCOPE:
        return
    from arena_autonomy.native_lidar_input import input_representation
    motion = node.motion
    if observation_scope == ESTIMATED_SCOPE:
        if motion is None or not estimated_environment(
                execution_environment=getattr(node, 'execution_environment', None),
                use_sim_time=node.get_parameter('use_sim_time').value,
                motion_verified=motion.verified, motion_time_policy=motion.timing_policy,
                motion_mode=motion.mode, lidar_input_representation=input_representation(node)):
            raise ValueError('estimated observation requires explicit physical runtime and unverified native timing')
        return
    if (node.get_parameter('use_sim_time').value is not True
            or node.hardware_inputs or motion is None or motion.mode != 'both'
            or motion.verified is not True or motion.timing_policy != 'verified_only'
            or input_representation(node) != 'native_simulation_v1'):
        raise ValueError('crop edge component requires verified-only native simulation with both motion')
