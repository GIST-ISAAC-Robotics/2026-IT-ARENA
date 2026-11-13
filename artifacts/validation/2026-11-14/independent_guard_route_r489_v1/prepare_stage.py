from pathlib import Path
import shutil
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
files=['src/arena_autonomy/arena_autonomy/lidar_safety.py','src/arena_autonomy/arena_autonomy/partial_observation.py','src/arena_autonomy/arena_autonomy/known_structure.py']
for name in files:
    for folder in ('base','stage'):
        p=A/folder/name;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,p)
p=A/'stage'/files[1];s=p.read_text(encoding='utf-8')
begin=s.index('        previous = self.previous if fresh else self.previous_distinct\n',s.index('    def evaluate('))
end=s.index('        # 좁은 정면 예외라도 장애물 기억을 먼저 계산한다.',begin)
body=s[begin:end]
call="""        failure = self._observe_missing_history(scan, points, history, now, speed, steering,
            previous_steering, wheelbase, length, width, source, fresh, valid, threat_unknown,
            origins, direction, stamps, step, required, report, finished)
        if failure is not None:
            return failure
"""
s=s[:begin]+call+s[end:]
body=body.replace("        if previous is None:\n            return", "        if previous is None and allow_empty_history:\n            self._remember(points, now, source, fresh)\n            return None\n        if previous is None:\n            return",1)
body=body.replace('        if now-previous[1] > .3:', '        if now-previous[1] > .3 and not allow_empty_history:',1)
method="""    def _observe_missing_history(self, scan, points, history, now, speed, steering,
            previous_steering, wheelbase, length, width, source, fresh, valid, threat_unknown,
            origins, direction, stamps, step, required, report, finished, *, allow_empty_history=False):
        # 기존 소실 위협/재관측/래치/기억 갱신 본문을 같은 순서로 공유한다.
        # 명시 guard는 없는 기억을 장애물로 만들지 않는다. 남은 실제 점은 나이로
        # 삭제하지 않고 기존 운동 이력으로 운반한다. 운동 무결성 오류는 허용하지 않는다.
"""+body+'        return None\n\n'
insert=s.index('    @staticmethod\n    def _excused_audit');s=s[:insert]+method+s[insert:]
needle='        # 선택형 structure_aware_v1만 설정한다.'
replacement="""        if getattr(self, 'command_observation_priority', False):
            from arena_autonomy.independent_guard_route import front_evidence
            evidence = front_evidence(scan, yaw)
            report['current_command_observation'] = evidence
            if not evidence['accepted']:
                return {**report, 'reason': evidence['reason']}
        # 선택형 structure_aware_v1만 설정한다."""
assert s.count(needle)==1;s=s.replace(needle,replacement)
needle='        self.clear_sources = 0\n        values = np.asarray(scan.ranges, dtype=float)'
replacement="""        if getattr(self, 'command_observation_priority', False):
            return self._current_command_observation(scan, points, history, now, speed,
                steering, previous_steering, wheelbase, length, width, source, fresh, excused, report, finished)
        self.clear_sources = 0
        values = np.asarray(scan.ranges, dtype=float)"""
assert s.count(needle)==1;s=s.replace(needle,replacement)
method="""    def _current_command_observation(self, scan, points, history, now, speed, steering,
            previous_steering, wheelbase, length, width, source, fresh, excused, report, finished):
        # 현재 반환/두 명령의 점유·제동은 호출 guard가 검사한다. 전역 결측 폭/chord,
        # 시험속도상한과 과거 자유 공간 warmup은 명령 허가 기준으로 사용하지 않는다.
        self.clear_sources = 0
        report['speed_cap_mps'] = None
        values = np.asarray(scan.ranges, dtype=float)
        angles = (scan_angles(scan)+history.config.lidar_yaw_rad+math.pi) % math.tau-math.pi
        valid = np.isfinite(values) & (values >= scan.range_min) & (values <= scan.range_max)
        missing = (np.abs(angles) <= math.radians(100)+1e-6) & ~valid
        if excused is not None:
            missing &= ~np.asarray(excused, dtype=bool)
        unknown = np.flatnonzero(missing)
        stamps = scan_times(scan)[unknown]
        poses = relative_poses(history, stamps, now)
        direction = angles[unknown]+poses[:,2]
        sensor = history.config
        c, sin = np.cos(poses[:,2]), np.sin(poses[:,2])
        origins = poses[:,:2]+np.column_stack((c*sensor.lidar_x_m-sin*sensor.lidar_y_m,
                                              sin*sensor.lidar_x_m+c*sensor.lidar_y_m))
        step = scan_max_gap(scan)
        required = max(.6, .16+speed*(now-source+.10)+speed*speed/4)
        failure = self._observe_missing_history(scan, points, history, now, speed, steering,
            previous_steering, wheelbase, length, width, source, fresh, valid, unknown,
            origins, direction, stamps, step, required, report, finished, allow_empty_history=True)
        if failure is not None:
            return failure
        return finished(allowed=not self.latched,
            reason='known_obstacle_latched' if self.latched else 'current_command_observed_returns')

"""
insert=s.index('    def _observe_missing_history');s=s[:insert]+method+s[insert:]
p.write_text(s,encoding='utf-8',newline='\n')
p=A/'stage'/files[2];s=p.read_text(encoding='utf-8')
needle="        for shadow in shadows:\n            radius = shadow['hidden_width_range_m']"
replacement="""        # 명시 guard는 가상 먼 그림자를 점유로 만들지 않는다. 위 실제 기억 갱신과
        # 아래 hidden-memory clearance/cap은 그대로 유지한다.
        for shadow in ([] if getattr(self, 'command_observation_priority', False) else shadows):
            radius = shadow['hidden_width_range_m']"""
assert s.count(needle)==1;s=s.replace(needle,replacement);p.write_text(s,encoding='utf-8',newline='\n')
p=A/'stage'/files[0];s=p.read_text(encoding='utf-8')
needle="        if traffic_recovery and policy != 'bounded_local_v4':"
replacement="""        guard_policy = str(self.declare_parameter('g1_guard_observation_policy', 'original_v1').value)
        if guard_policy not in ('original_v1', 'selected_r181'):
            raise ValueError('unknown guard observation policy')
        self.command_observation_priority = traffic_recovery and guard_policy == 'selected_r181'
        if traffic_recovery and policy != 'bounded_local_v4':"""
assert s.count(needle)==1;s=s.replace(needle,replacement)
needle='            if self.observation_input_scope == ESTIMATED_SCOPE:'
replacement="""            if self.command_observation_priority:
                self.partial_policy.command_observation_priority = True
            if self.observation_input_scope == ESTIMATED_SCOPE:"""
assert s.count(needle)==1;s=s.replace(needle,replacement)
needle="""            clearance = swept_limit(points, output.drive.steering_angle, self.wheelbase,
                                    length=self.length, width=self.width)"""
replacement="""            if getattr(self, 'command_observation_priority', False):
                # planner 경로/허가는 받지 않고 실제 반환과 현재/직전 명령만 검사한다.
                clearance = min(swept_limit(points, angle, self.wheelbase,
                    length=self.length, width=self.width) for angle in
                    sorted({float(output.drive.steering_angle), float(self.previous_safe_steering)}))
            else:
                clearance = swept_limit(points, output.drive.steering_angle, self.wheelbase,
                                        length=self.length, width=self.width)"""
assert s.count(needle)==1;s=s.replace(needle,replacement);p.write_text(s,encoding='utf-8',newline='\n')
print('stage source prepared; common unchanged')
