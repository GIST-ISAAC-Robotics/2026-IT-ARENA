"""별도 실행 파일에서만 선택하는 G2 동역학 시험. 기본 LocalPursuit는 변경하지 않는다."""
from pathlib import Path
from arena_autonomy.local_pursuit import LocalPursuit
from arena_autonomy.bypass_entry_follow import BypassEntryFollow,BypassExecutionConfig
from arena_autonomy.follow_trace import FollowTrace, freeze
from arena_vehicle_interface.node_lifecycle import run_node


STATUS_KEYS=('reason','behavior','mode','passing_enabled','simulation_trial_only','automatic_race_mode_selection',
    'output_requires_independent_guard','source_first_s','reference_stamp_s','source_epoch','input_error',
    'guide_mode','entry_phase','intent_direction','rejoin_confirmations','center_confirmations',
    'outer_wall_fresh_confirmations','boundary_confirmations','control_authorized','unknown_filled',
    'full_bypass_verified','road_surface_verified','selected')


def status_detail(detail):
    """/autonomy/status용 요약. 전체 판단은 LocalPursuit.command가 schema4 기록에 먼저 남긴다.

    후보 경로점·NumPy 값은 기존 상태 JSON(allow_nan=False)에 넣지 않는다. 명령 값은 바꾸지 않는다.
    """
    compact={key:detail[key] for key in STATUS_KEYS if key in detail}
    candidates=detail.get('candidates') or []
    compact['candidate_count']=len(candidates)
    compact['eligible_candidates']=sum(bool(row.get('eligible')) for row in candidates)
    if len(candidates)==1:
        compact['guided_candidate']={key:candidates[0].get(key) for key in
            ('reason','target_speed_mps','command_clearance_m','steering_rad','preview_return_progress_m')}
    for key,fields in (('path_reanchor',('applied','reason')),
                       ('rejoin_band',('supported','front_unobserved','rear_unobserved','current_occupied_returns'))):
        if isinstance(detail.get(key),dict):
            compact[key]={field:detail[key].get(field) for field in fields}
    return freeze(compact)


class G2EntryNode(LocalPursuit):
    def __init__(self):
        super().__init__()
        enabled=bool(self.declare_parameter('g2_entry_simulation_trial',False).value)
        trace_path=str(self.declare_parameter('g2_trace_path','').value)
        if not enabled or self.get_parameter('use_sim_time').value is not True:
            raise ValueError('G2 requires explicit simulation-only activation')
        if (not self.g1_enabled or self.follow is None or self.r0_plus_enabled or self.m3_execution_enabled
                or self.follow_trace is not None or not trace_path or not Path(trace_path).is_absolute()):
            raise ValueError('G2 requires G1 sensor configuration, its own absolute trace and no competing controller')
        self.follow=BypassEntryFollow(self.follow.config,BypassExecutionConfig(),allow_simulation_trial=True)
        self.follow_trace=FollowTrace(trace_path,self.motion.history.config,self.follow.config,
            mode='g2_entry_trial',execution_config=self.follow.execution)
        self.get_logger().info('G2 explicit sensor entry trial; publishes /drive through independent safety')

    def command(self, points):
        # 기존 센서 입력·판단 기록·예외 처리를 그대로 쓰고 상태 발행용 detail만 줄인다.
        target,steering,detail=super().command(points)
        return target,steering,status_detail(detail)


def main(args=None):
    run_node(G2EntryNode,args=args)
