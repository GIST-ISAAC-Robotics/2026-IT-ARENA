"""G1/G2 전환을 별도 실행 파일에서만 선택한다. 기존 launch의 기본값은 유지한다."""
from pathlib import Path

from arena_autonomy.local_pursuit import LocalPursuit
from arena_autonomy.bypass_entry_follow import BypassExecutionConfig, CORRIDOR_POLICIES
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig, PREPARATION_POLICIES, PERSISTENCE_WINDOWS
from arena_autonomy.follow_trace import FollowTrace, freeze
from arena_autonomy.g2_entry_node import status_detail as g2_status
from arena_vehicle_interface.node_lifecycle import run_node


def status_detail(detail):
    result = {key:detail[key] for key in ('mode','handoff_state','mode_before','behavior','reason',
        'simulation_trial_only','automatic_race_mode_selection','output_requires_independent_guard','input_error')
        if key in detail}
    if isinstance(detail.get('normal'),dict):
        result['normal'] = {key:detail['normal'][key] for key in
            ('reason','behavior','selector_mode','selector_choice','selector_reason') if key in detail['normal']}
    if isinstance(detail.get('bypass'),dict):
        result['bypass'] = g2_status(detail['bypass'])
    if isinstance(detail.get('persistence'),dict):
        result['persistence'] = detail['persistence']
    if isinstance(detail.get('preparation'),dict):
        result['preparation'] = {key:detail['preparation'][key] for key in
            ('policy','applied','reason') if key in detail['preparation']}
    if isinstance(detail.get('follow_entry_space'),dict):
        result['follow_entry_space'] = {key:detail['follow_entry_space'][key] for key in
            ('applied','reason','rear_axle_to_patch_m','reserve_m','cap_mps',
             'hold_age_s','maximum_age_s','current_front_evidence','bypass_permission') if key in detail['follow_entry_space']}
    return freeze(result)


class G1G2Node(LocalPursuit):
    def create_follow_trace(self, trace_path):
        enabled = self.declare_parameter('g1_g2_simulation_trial',False).value
        if enabled is not True or self.get_parameter('use_sim_time').value is not True:
            raise ValueError('G1/G2 handoff requires explicit simulation-only activation')
        # 기본은 기존 즉시 준비 감속이다. 새 관측 근거 조건은 이름으로 명시할 때만 쓴다.
        policy = self.declare_parameter('g1_g2_preparation_policy','front_patch_v0').value
        if type(policy) is not str or policy not in PREPARATION_POLICIES:
            raise ValueError('registered G1/G2 preparation policy required')
        window = self.declare_parameter('g1_g2_persistence_window','compare_then_trim_v0').value
        if type(window) is not str or window not in PERSISTENCE_WINDOWS:
            raise ValueError('registered G1/G2 persistence window required')
        entry_policy = self.declare_parameter('g1_g2_entry_policy','registered_v0').value
        corridor = self.declare_parameter('g1_g2_corridor_policy','off').value
        if type(entry_policy) is not str or type(corridor) is not str or corridor not in CORRIDOR_POLICIES:
            raise ValueError('typed G1/G2 entry and corridor policies required')
        reserve = self.declare_parameter('g1_g2_follow_entry_space',False).value
        reacquire = self.declare_parameter('g1_g2_reacquire_policy','positive_command_v0').value
        preparation_hold = self.declare_parameter('g1_g2_preparation_hold','off').value
        if type(reserve) is not bool or type(reacquire) is not str or type(preparation_hold) is not str:
            raise ValueError('typed G1/G2 follow space and reacquire options required')
        if (not self.g1_enabled or self.follow is None or self.r0_plus_enabled or self.m3_execution_enabled
                or not trace_path or not Path(trace_path).is_absolute()):
            raise ValueError('G1/G2 handoff requires G1 sensors, its own absolute trace and no competing controller')
        self.follow = BypassHandoffFollow(self.follow.config,HandoffExecutionConfig(preparation_policy=policy,
            persistence_window_policy=window,follow_entry_space_enabled=reserve,reacquire_policy=reacquire,
            entry_policy=entry_policy,preparation_hold_policy=preparation_hold,
            bypass=BypassExecutionConfig(corridor_policy=corridor)),
            allow_simulation_trial=True)
        trace = FollowTrace(trace_path,self.motion.history.config,self.follow.config,
            mode='g1_g2_handoff_trial',execution_config=self.follow.execution)
        self.get_logger().info('G1/G2 sensor handoff trial (preparation '+policy+'); /drive through independent safety')
        return trace

    def command(self, points):
        target, steering, detail = super().command(points)
        return target, steering, status_detail(detail)


def main(args=None):
    run_node(G1G2Node,args=args)
