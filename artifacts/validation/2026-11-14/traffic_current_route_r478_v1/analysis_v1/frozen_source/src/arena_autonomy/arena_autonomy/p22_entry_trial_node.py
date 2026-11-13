"""명시 선택한 단독 P22 입구 시험. 경기 기본값이나 출구 합류 구현이 아니다."""
from pathlib import Path

from arena_autonomy.local_pursuit import LocalPursuit
from arena_autonomy.follow_trace import FollowTrace, controller_mode
from arena_autonomy.p22_trial import P22Entry
from arena_autonomy.p22_route_trial import P22Route
from arena_vehicle_interface.node_lifecycle import run_node


class P22EntryNode(LocalPursuit):
    def create_follow_trace(self, trace_path):
        enabled = self.declare_parameter('p22_entry_simulation_trial', False).value
        if enabled is not True or self.get_parameter('use_sim_time').value is not True:
            raise ValueError('P22 entry requires explicit simulation-only activation')
        if (not self.g1_enabled or self.follow is None or self.r0_plus_enabled or self.m3_execution_enabled
                or not trace_path or not Path(trace_path).is_absolute() or self.vision_pipeline is None):
            raise ValueError('P22 entry requires G1, process vision and its own absolute decision trace')
        route = self.declare_parameter('p22_route_simulation_trial', False).value
        self.follow = (P22Route if route is True else P22Entry)(self.follow)
        self.p22_recorded_frame = None
        return FollowTrace(trace_path, self.motion.history.config, self.follow.config, mode=controller_mode(self.follow))

    def reset_follow(self, reason):
        if not isinstance(getattr(self, 'follow', None), P22Entry):
            return super().reset_follow(reason)
        force = reason in ('autonomy_reset', 'legacy_reset')
        self.follow.reset(force=force)
        # 자동 초기화 뒤에도 같은 영상이 다시 입력되면 재생에 그 사실을 남긴다.
        self.p22_recorded_frame = None
        if getattr(self, 'follow_trace', None) is not None:
            self.follow_trace.emit('reset', {'reason': reason, 'force': force})

    def command(self, points):
        pipeline = self.vision_pipeline
        result = pipeline.last_result
        # 영상 취득 시각을 그대로 기록한다. 상태 발행/명령 시각을 대신 쓰지 않는다.
        frame = dict(ids=list(pipeline.ids) if not pipeline.fault else [],
                     stamp=result.get('stamp') if result and not pipeline.fault else None,
                     now=self.now_s(), epoch=pipeline.epoch)
        self.follow.frame(**frame)
        key = (frame['epoch'], frame['stamp'], tuple(frame['ids']))
        if key != self.p22_recorded_frame:
            self.follow_trace.emit('p22_frame', frame)
            self.p22_recorded_frame = key
        target, steering, detail = super().command(points)
        # 원 경로/입력은 결정 기록에 보존하고 ROS 상태는 작게 유지한다.
        detail = dict(detail)
        if 'p22_trial' in detail:
            detail['p22_trial'] = {k:v for k,v in detail['p22_trial'].items() if k != 'path_points'}
        return target, steering, detail


def main(args=None):
    run_node(P22EntryNode, args=args)
