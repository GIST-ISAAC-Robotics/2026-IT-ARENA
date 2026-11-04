"""관측 이동 근거 옵션 연결 1회. 줄 단위 바이트 패치로 줄바꿈을 보존한다. 실행 뒤 삭제한다."""
from pathlib import Path


def after(path, anchor, lines, before=False):
    p = Path(path)
    data = p.read_bytes()
    a = anchor.encode('utf8')
    assert data.count(a) == 1, anchor
    start = data.index(a)
    begin = data.rfind(b'\n', 0, start)+1
    end = data.index(b'\n', start)+1
    eol = b'\r\n' if data[end-2:end] == b'\r\n' else b'\n'
    block = b''.join(line.encode('utf8')+eol for line in lines)
    p.write_bytes(data[:begin]+block+data[begin:] if before else data[:end]+block+data[end:])


def sub(path, old, new):
    p = Path(path)
    data = p.read_bytes()
    a = old.encode('utf8')
    assert data.count(a) == 1, old
    p.write_bytes(data.replace(a, new.encode('utf8')))


F = 'src/arena_autonomy/arena_autonomy/'
L = F+'local_follow.py'
after(L, '    released_rear_sweep_trial: bool = False', ['    static_side_release_trial: bool = False'])
after(L, "            raise ValueError('released rear sweep requires traffic recovery')", [
    "        if (type(self.static_side_release_trial) is not bool or",
    "                (self.static_side_release_trial and not self.pass_release_trial)):",
    "            raise ValueError('static side release requires pass release trial')",
])
sub(L, "if k not in ('corridor_arc_agreement',", "if k not in ('static_side_release_trial', 'corridor_arc_agreement',")

P = F+'pass_release_trial.py'
after(P, "        self.lat = lateral_config(follow_config)", [
    "        self.static_side = ({name: StaticSideTracker() for name, _ in SIDES}",
    "                            if follow_config.static_side_release_trial else None)",
])
after(P, "        self.sides = {name: self._side() for name, _ in SIDES}", ["        self._reset_static_side()"])
after(P, "        return {'seen': False, 'present': False, 'clear': 0, 'passed': False, 'vanish': None, 'last_present': None}", [
    "",
    "    def _reset_static_side(self):",
    "        for tracker in (self.static_side or {}).values():",
    "            tracker.reset()",
    "",
    "    def _world_fixed(self, name, now, now_side, evidence):",
    "        \"\"\"자차 이동량만큼 물러나는 측방 반환은 측방 위협에서 뺀다. 경로 점유 검사는 그대로다.\"\"\"",
    "        if self.static_side is None or not self.static_side[name].observe(now, now_side['frontmost_ds_m']):",
    "            return False",
    "        self.sides[name].update(self._side())",
    "        evidence[name]['world_fixed_return'] = True",
    "        return True",
])
after(P, "        \"\"\"시간 연결을 잃었다. 위협을 본 적이 있으면 양쪽을 unknown으로 두고 새 통과 증거를 요구한다.\"\"\"", [
    "        self._reset_static_side()",
])
after(P, "        fresh = self.last_source is None or source > self.last_source", [
    "        for tracker in (self.static_side or {}).values():",
    "            tracker.advance(speed, dt)",
])
after(P, "                evidence[name] = dict(now_side)", [
    "                if self._world_fixed(name, now, now_side, evidence):",
    "                    continue",
])
after(P, "from arena_autonomy.lidar_motion", ["from arena_autonomy.static_side_return import StaticSideTracker"]) \
    if b'from arena_autonomy.lidar_motion' in Path(P).read_bytes() else None

H = F+'bypass_handoff.py'
after(H, "FRONT_PATCH_POLICIES = ('contiguous_three_v0', 'sparse_returns_v1')", [
    "# 정지 중 보정 뒤 이동은 0–2 cm였고 재출발 첫 스캔은 2.8 cm였다. 이 스캔에서 우회를 시작하지 않는다.",
    "ENTRY_FORWARD_DRIFT_M = .02",
])
after(H, "    relative_yield_policy: str = 'off'", [
    "    # 전방 조각이 전진하기 시작한 스캔의 우회 진입을 막는 선택형 근거. 정지 판정이 아니다.",
    "    entry_motion_gate: bool = False",
])
after(H, "        self.forward_rate_mps = 0.", ["        self.forward_drift_m = 0."])
after(H, "        if deviations and max(deviations) > self.config.maximum_patch_displacement_m:", [
    "        self.forward_drift_m = float(deltas[0][0]) if deltas else 0.",
], before=True)
sub(H, "if (persistence['ready'] and speed <= self.bypass.config.max_speed_mps",
    "if (persistence['ready'] and not self._front_patch_advancing() and speed <= self.bypass.config.max_speed_mps")
after(H, "    def _straight_reference(self,scene,config=None):", [
    "    def _front_patch_advancing(self):",
    "        return self.execution.entry_motion_gate and self.monitor.forward_drift_m > ENTRY_FORWARD_DRIFT_M",
    "",
], before=True)

N = F+'g1_g2_node.py'
after(N, "        rear_sweep = self.declare_parameter('g1_g2_released_rear_sweep_trial',False).value", [
    "        observed_motion = self.declare_parameter('g1_g2_observed_motion_gates',False).value",
])
sub(N, "config = replace(self.follow.config,released_rear_sweep_trial=rear_sweep)",
    "config = replace(self.follow.config,released_rear_sweep_trial=rear_sweep,static_side_release_trial=observed_motion)")
sub(N, "            relative_yield_policy=relative_yield,", "            relative_yield_policy=relative_yield,entry_motion_gate=observed_motion,")

S = 'src/arena_bringup/launch/simulation.launch.py'
after(S, '        rear_sweep = LaunchConfiguration("evaluation_handoff_rear_sweep").perform(context) == "true"', [
    '        observed_motion = LaunchConfiguration("evaluation_handoff_observed_motion").perform(context) == "true"',
])
after(S, "                g1_g2_released_rear_sweep_trial=rear_sweep,", ["                g1_g2_observed_motion_gates=observed_motion,"])
after(S, '            DeclareLaunchArgument("evaluation_handoff_yield_progress", default_value="false", choices=["false", "true"],', [
    '            DeclareLaunchArgument("evaluation_handoff_observed_motion", default_value="false", choices=["false", "true"],',
    '                                  description="세계 고정 측방 반환과 전진 시작 조각을 구별하는 시험 후보"),',
], before=True)

G = 'scripts/g1_handoff_eval.py'
sub(G, "def launch_arguments(mode, reserve, launch, recorded, *, progress=False, rear_sweep=False):",
    "def launch_arguments(mode, reserve, launch, recorded, *, progress=False, rear_sweep=False, observed_motion=False):")
after(G, "    if rear_sweep:extra.append('evaluation_handoff_rear_sweep:=true')", [
    "    if observed_motion:extra.append('evaluation_handoff_observed_motion:=true')",
])

V = 'scripts/validate_g1_traffic.py'
after(V, "    parser.add_argument('--handoff-rear-sweep', action='store_true', help='통과 후 저속 후반부 이동 시험')", [
    "    parser.add_argument('--handoff-observed-motion', action='store_true',",
    "                        help='세계 고정 측방 반환과 전진 시작 조각을 구별하는 선택형 후보. 공식 교통 경로 전용.')",
])
sub(V, "progress=args.handoff_yield_progress, rear_sweep=args.handoff_rear_sweep)",
    "progress=args.handoff_yield_progress, rear_sweep=args.handoff_rear_sweep, observed_motion=args.handoff_observed_motion)")
after(V, "    if args.team_signal_course and args.official_signal_course:", [
    "    if args.handoff_observed_motion and not args.official_signal_course:",
    "        raise ValueError('관측 이동 후보는 공식 교통 경로에서만 선택한다')",
], before=True)

sub('scripts/traffic_merge.py', "'static_heading_matches': bool(static_errors) and max(static_errors) <= rules['static_heading_error_max_rad'],",
    "'static_heading_matches': (max(static_errors) <= rules['static_heading_error_max_rad'] if static_errors\n"
    "                                   else all(row['speed_mps'] > 0 for row in case['schedule'])),")

E = 'artifacts/validation/2026-11-02/official_forward_v1/scene.sh'
sub(E, '--record-follow-decisions --output "$PWD/$A/$NAME"', '--record-follow-decisions ${3:-} --output "$PWD/$A/$NAME"')
