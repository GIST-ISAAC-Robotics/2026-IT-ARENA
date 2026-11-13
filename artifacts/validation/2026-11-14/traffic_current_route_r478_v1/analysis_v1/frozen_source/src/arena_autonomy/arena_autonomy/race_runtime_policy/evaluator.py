"""명시한 두 계산 자식 안에서만 설치하는 기존 정책 묶음."""
from . import observation_policy
from arena_autonomy.crop_edge_selection import (SIMULATION_SCOPE, ESTIMATED_SCOPE,
    validate_observation_scope, timing_matches)

POLICY = 'cad_runtime_r377_v1'


def supported(output):
    return observation_policy.supported(output)


class Evaluator:
    def __init__(self, stage, header):
        if header.get('runtime_policy') != POLICY or stage not in ('plan', 'current'):
            raise ValueError('explicit registered runtime policy and stage required')
        self.stage = stage
        self.vehicle_marker_profile = header.get('vehicle_marker_profile')
        self.vehicle_marker_clock_domain = header.get('vehicle_marker_clock_domain')
        if self.vehicle_marker_profile is not None:
            from arena_autonomy.vehicle_marker_geometry import validate_profile
            validate_profile(self.vehicle_marker_profile)
            if self.vehicle_marker_clock_domain not in ('ros_sim','ros_system'):
                raise ValueError('explicit vehicle marker clock domain required')
        self.time_policy = header.get('runtime_time_policy', 'verified_only')
        self.observation_scope = validate_observation_scope(header.get('observation_input_scope', SIMULATION_SCOPE))
        if self.time_policy not in ('verified_only', 'host_estimated_v1'):
            raise ValueError('explicit runtime acquisition policy required')
        from arena_autonomy.runtime_node_backend import validate_corner_selection
        corner = header.get('current_corner_budget', 'off')
        environment = header.get('current_corner_environment', {}) if corner != 'off' else {}
        if type(environment) is not dict:
            raise ValueError('explicit current corner environment mapping required')
        self.corner_budget = validate_corner_selection(corner,
            runtime_selected=(header.get('runtime_policy') == POLICY
                and header.get('numeric_execution') == 'native_v1'
                and environment.get('backend') == 'native2slot_v1'),
            use_sim_time=environment.get('use_sim_time'),
            motion_verified=environment.get('motion_timing_verified'), motion_time_policy=self.time_policy,
            observation_scope=self.observation_scope,
            execution_environment=environment.get('execution_environment'),
            lidar_input_representation=environment.get('lidar_input_representation'),
            motion_mode=environment.get('motion_mode'))
        from arena_autonomy.numeric_kernels import configure
        from arena_autonomy.decision_process import make_controller
        configure(header.get('numeric_execution', 'python'))
        self.plan_scope = None
        if stage == 'plan':
            from . import terminal_plan, released_plan
            terminal_plan.install()
            self.plan_scope = released_plan.enabled()
            self.plan_scope.__enter__()
        else:
            from . import released_relative
            self.current_policy = released_relative
        try:
            self.follow, self.motion = make_controller(header)
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.plan_scope is not None:
            from . import terminal_plan
            self.plan_scope.__exit__(None,None,None)
            self.plan_scope = None
            terminal_plan.uninstall()

    def reset(self, reason):
        from arena_autonomy.follow_trace import replay_reset
        replay_reset(self.follow, {'reason': reason})

    def __call__(self, data):
        from arena_autonomy.follow_trace import freeze, replay_call, restore_call
        from arena_autonomy.lidar_motion import MotionUnavailable
        from arena_autonomy.scene_motion import motion_scene
        current = data if self.stage == 'plan' else data['current']
        if current.get('timing_policy', 'verified_only') != self.time_policy:
            raise ValueError('input acquisition policy differs from selected runtime evaluator')
        scope = getattr(self, 'observation_scope', SIMULATION_SCOPE)
        if self.corner_budget:
            if scope == ESTIMATED_SCOPE:
                raw = current.get('scan', {})
                if (raw.get('representation')!='native_measurements_v1' or not timing_matches(
                        scope,current.get('timing_verified'),current.get('timing_policy'),
                        raw.get('measurements',{}).get('time_basis'))):
                    raise ValueError('estimated observation requires unchanged false/host-estimated input timing')
            elif current.get('timing_verified') is not True:
                raise ValueError('current corner budget requires verified input timing')
        if self.stage == 'plan':
            if self.vehicle_marker_profile is None:
                return freeze(replay_call(self.follow, data, self.motion))
            from arena_autonomy.vehicle_marker_occupancy import marker_scope, apply_plan
            _, _, history = restore_call(data, self.motion)
            with marker_scope(data, history, self.follow.config, self.vehicle_marker_profile,
                              self.vehicle_marker_clock_domain):
                output = replay_call(self.follow, data, self.motion)
                return freeze(apply_plan(output, data, history, self.follow.config,
                    self.vehicle_marker_profile, self.vehicle_marker_clock_domain))
        info, scan, history = restore_call(current, self.motion)
        try:
            scene = motion_scene(scan, history, info['reference_s'], timing_verified=info['timing_verified'],
                                 epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'],
                                 timing_policy=self.time_policy)
            result = self.current_policy.rebase(data['output'], data['submitted'], current, scene,
                                               history, self.follow.config, require_current_region=True)
            if self.corner_budget:
                result = self.apply_corner(result, data, info, scene, history)
                from .crop_current_candidate import try_from_current_result
                result = try_from_current_result(result, data, scene, history, self.follow.config,
                    registered_environment=self.corner_budget, core_rebase=self.current_policy.core_policy.rebase,
                    **(dict(observation_scope=scope) if scope==ESTIMATED_SCOPE else {}))
        except (MotionUnavailable, ValueError) as error:
            result = dict(speed_mps=0., steering_rad=info['previous_steering_rad'],
                          reference_s=info['reference_s'], reason='rebase_input_stop', input_error=str(error),
                          independent_guard_required=True, runtime_authorized=False)
        if result is None:
            raise ValueError('unsupported current recheck')
        if self.vehicle_marker_profile is not None:
            from arena_autonomy.vehicle_marker_occupancy import apply_current
            result = apply_current(result, info, history, self.follow.config,
                self.vehicle_marker_profile, self.vehicle_marker_clock_domain)
        return freeze(result)

    def apply_corner(self, before, data, info, scene, history):
        """계산된 baseline을 받고 eligible일 때만 원 단일 제안/최종 검사를 추가한다."""
        from . import corner_candidate, range_mask_component
        from arena_autonomy.local_scene import blind_spec
        cfg = self.follow.config
        scope = getattr(self, 'observation_scope', SIMULATION_SCOPE)
        if scope==ESTIMATED_SCOPE and (before is None or before['speed_mps']!=0):
            return before
        if not corner_candidate.eligible(before, data['output'], data['current'], cfg, True):
            return before
        profile = info['rear_half_angle_deg']
        sectors = blind_spec(profile)[1]

        def proof(s, raw, q, padding, length, width, speed, target):
            return range_mask_component.bounded_adjacent_structure(
                s, raw, q, padding, length, width, speed, target, sectors,
                mask_profile=profile, motion_epoch=info['epoch'], timing_verified=info['timing_verified'],
                **(dict(observation_scope=scope,timing_policy=self.time_policy) if scope==ESTIMATED_SCOPE else {}))

        return corner_candidate.try_from_current_result(before, data['output'], data['submitted'],
            data['current'], scene, history, cfg, core_rebase=self.current_policy.core_policy.rebase,
            proof_callback=proof, require_current_region=True,
            **(dict(target_cap=.1) if scope==ESTIMATED_SCOPE else {}))


def evaluator_factory(stage, header):
    return Evaluator(stage, header)
