"""개발 중 일회성 소스 편집 스크립트(결과 불변 최적화). 실행 증거가 아니다."""
from pathlib import Path

p = Path('src/arena_autonomy/arena_autonomy/m3_execution_trial.py')
s = p.read_text(encoding='utf-8')


def rep(old, new):
    global s
    assert s.count(old) == 1, old[:120]
    s = s.replace(old, new)


rep("""from dataclasses import dataclass, replace
import math
""", """from collections import deque
from dataclasses import dataclass, replace
import math
""")
rep("""from arena_autonomy.lateral_intent import (LateralIntentConfig, ObservationGrid, PathFrame, ego_state,
    cell_coordinates, corridor_reference, side_threats, offset_candidate, surface_check, lateral_clearance_check,
    committed_progress_m, wheel_contact_corners, GEOMETRY_EPS_M)
from arena_autonomy.observed_lateral_trial import path_observation_queries
from arena_autonomy.observation_support import depth_support, source_valid, RecentScenes
from arena_autonomy.m3_observed_follow import command_arc""",
"""from arena_autonomy.lateral_intent import (LateralIntentConfig, ObservationGrid, PathFrame, ego_state,
    cell_coordinates, corridor_reference, side_threats, offset_candidate, surface_check, lateral_clearance_check,
    committed_progress_m, wheel_contact_corners, compact_return_mask, GEOMETRY_EPS_M, SIDES)
from arena_autonomy.observed_lateral_trial import path_observation_queries
from arena_autonomy.observation_support import depth_support, source_valid
from arena_autonomy.scene_motion import relative_poses
from arena_autonomy.m3_observed_follow import command_arc""")

start = s.index('def support(scene, recent, queries, cfg, execution):')
end = s.index('def body_queries(')
s = s[:start] + '''class SceneHistory:
    """같은 세대·0.3 s 안의 이전 장면(원래 취득 좌표 그대로). 같은 스캔으로 갱신하지 않고 역행 시 비운다."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.scenes = deque(maxlen=3)
        self.last_ref = self.epoch = None

    def update(self, scene, history):
        """현재 장면을 기록하고, 현재 기준 좌표로의 (dx, dy, yaw)와 함께 이전 장면 목록을 반환한다."""
        if (not source_valid(scene) or (self.last_ref is not None and scene.t_ref_s < self.last_ref) or
                (self.scenes and scene.source_first_s < self.scenes[-1].source_first_s)):
            self.reset()
            return []
        if self.epoch is not None and self.epoch != scene.source_epoch:
            self.reset()
        self.last_ref, self.epoch = scene.t_ref_s, scene.source_epoch
        if not self.scenes or scene.source_first_s > self.scenes[-1].source_first_s:
            self.scenes.append(scene)
        while self.scenes and scene.t_ref_s-self.scenes[0].source_first_s >= .3:
            self.scenes.popleft()
        frames = []
        for old in reversed(self.scenes):
            if old.source_first_s >= scene.source_first_s or old.source_epoch != scene.source_epoch:
                continue
            try:
                dx, dy, yaw = relative_poses(history, [old.t_ref_s], scene.t_ref_s)[0]
            except MotionUnavailable:
                continue
            frames.append((old, (float(dx), float(dy), float(yaw))))
        return frames


def support(scene, frames, queries, cfg, execution):
    """현재 장면 지지 + (선택) 마스크 원인 질의의 이전 장면 지지.

    이전 장면 광선을 현재 좌표로 옮기는 대신 질의점을 이전 장면의 기준 좌표로 역변환한다(같은 강체
    관계). 여유는 현재 시각 기준 광선 나이(기존 0.5 m/s 이동 상계)로 계산한다.
    """
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    pad = ray_padding(scene, cfg, execution.history_motion_bound_mps)
    ok = depth_support(scene, q, pad) if len(q) else np.zeros(0, dtype=bool)
    detail = {'queries': int(len(q)), 'current_unsupported': int(np.sum(~ok)), 'masked_candidates': 0,
              'history_supported': 0, 'history_sources': 0}
    if execution.masked_history_support and np.any(~ok) and frames:
        missing = np.flatnonzero(~ok)
        masked = masked_only(scene, q[missing], pad)
        detail['masked_candidates'] = int(masked.sum())
        remaining = missing[masked]
        for old, (dx, dy, yaw) in frames:
            if not len(remaining):
                break
            c, sn = math.cos(yaw), math.sin(yaw)
            local = (q[remaining]-[dx, dy])@np.array([[c, -sn], [sn, c]])
            old_pad = np.array([.025+cfg.budget.range_bias_m+execution.history_motion_bound_mps*(
                scene.t_ref_s-r.acquired_at_s) for r in old.rays])
            accepted = depth_support(old, local, old_pad)
            ok[remaining[accepted]] = True
            detail['history_supported'] += int(accepted.sum())
            detail['history_sources'] += 1
            remaining = remaining[~accepted]
    detail.update(supported=bool(np.all(ok)), unsupported=int(np.sum(~ok)))
    return detail


''' + s[end:]

rep("""class ExecutionLateralTrial:""", '''class ReturnGrid:
    """측방 반환이 하나도 없을 때 쓰는 반환 정보. 이때 side_threats는 칸 상태/좌표를 읽지 않는다."""

    def __init__(self, scene, cfg):
        rays = [r for r in scene.rays if r.status == 'return']
        self.returns = np.asarray([r.hit_xy_m for r in rays], dtype=float).reshape(-1, 2)
        self.ages = np.asarray([scene.t_ref_s-r.acquired_at_s for r in rays], dtype=float)
        self.compact_returns, self.return_components = compact_return_mask(scene, cfg)


def side_points(frame, state, grid, cfg):
    """side_threats와 같은 창·벽·횡위치·compact 조건의 측방 반환 개수(좌, 우)."""
    if not len(grid.returns):
        return 0, 0
    s, l = frame.project(grid.returns)
    ds = s-state['s0']
    window = (ds >= -cfg.band_behind_m) & (ds <= cfg.band_ahead_m)
    wall = np.zeros(len(l), dtype=bool)
    for _, sign in SIDES:
        wall |= np.abs(l-sign*cfg.wall_offset_m) <= cfg.wall_tolerance_m
    interior = np.abs(l) < cfg.wall_offset_m-cfg.wall_tolerance_m
    return tuple(int(np.sum(window & interior & ~wall & (sign*(l-state['d0']) >= state['half_span_m'])
                            & grid.compact_returns)) for _, sign in SIDES)


class ExecutionLateralTrial:''')

rep("""    def step(self, scene, recent, path, speed, follow_padding):""",
    """    def step(self, scene, frames, path, speed, follow_padding):""")
rep("""        state = ego_state(frame, cfg)
        grid = ObservationGrid(scene, cfg)
        corridor = corridor_reference(frame, grid.returns, state['s0']-cfg.band_behind_m, state['s0']+cfg.horizon_m, cfg)
        s, l = cell_coordinates(grid, frame)
        threats = side_threats(frame, state, grid, cfg, s, l, grid.observed_free())""",
"""        state = ego_state(frame, cfg)
        light = ReturnGrid(scene, cfg)
        if any(side_points(frame, state, light, cfg)):
            grid = ObservationGrid(scene, cfg)
            s, l = cell_coordinates(grid, frame)
            observed = grid.observed_free()
        else:
            # 측방 반환이 없으면 side_threats는 칸 좌표/상태를 읽지 않는다(같은 결과, 계산 생략).
            grid, s, l, observed = light, None, None, None
        corridor = corridor_reference(frame, grid.returns, state['s0']-cfg.band_behind_m, state['s0']+cfg.horizon_m, cfg)
        threats = side_threats(frame, state, grid, cfg, s, l, observed)""")
rep("""        checks = {name: plan_check(scene, recent, c, context, cfg, self.execution, speed)
                  for name, c in candidates.items()}
        out['checks'] = checks
        for key in order:
            if checks[key]['eligible_for_trial']:
                if key == 'center' and abs(state['d0']) < .005:
                    self.active = False
                return {**out, 'mode': key.upper()+'_TRIAL', 'selected': key, 'path': candidates[key]['path']}
        return {**out, 'mode': 'HOLD_STOP_TRIAL', 'path': None}""",
"""        # 우선순위대로 평가하고 첫 통과 후보에서 멈춘다(선택 결과는 전체 평가와 같다).
        checks = {}
        out['checks'] = checks
        for key in order:
            checks[key] = plan_check(scene, frames, candidates[key], context, cfg, self.execution, speed)
            if checks[key]['eligible_for_trial']:
                if key == 'center' and abs(state['d0']) < .005:
                    self.active = False
                return {**out, 'mode': key.upper()+'_TRIAL', 'selected': key, 'path': candidates[key]['path']}
        return {**out, 'mode': 'HOLD_STOP_TRIAL', 'path': None}""")
for old, new in (
        ("def arc_checks(scene, recent, reference_path", "def arc_checks(scene, frames, reference_path"),
        ("    observation = support(scene, recent, np.unique(np.vstack(queries), axis=0), cfg, execution)",
         "    observation = support(scene, frames, np.unique(np.vstack(queries), axis=0), cfg, execution)"),
        ("def presteer_support(scene, recent, angles, cfg, execution):",
         "def presteer_support(scene, frames, angles, cfg, execution):"),
        ("    return support(scene, recent, q, cfg, execution)", "    return support(scene, frames, q, cfg, execution)"),
        ("def plan_check(scene, recent, candidate, context, cfg, execution, speed):",
         "def plan_check(scene, frames, candidate, context, cfg, execution, speed):"),
        ("    observation = support(scene, recent, q, cfg, execution)\n    result.update(",
         "    observation = support(scene, frames, q, cfg, execution)\n    result.update("),
        ("        self.recent = RecentScenes()", "        self.recent = SceneHistory()"),
        ("trial = self.lateral.step(scene, recent, baseline[2].get('path_points'), plan_speed, padding)",
         "trial = self.lateral.step(scene, frames, baseline[2].get('path_points'), plan_speed, padding)"),
        ("envelope = arc_checks(scene, recent, baseline[2]['path_points']",
         "envelope = arc_checks(scene, frames, baseline[2]['path_points']"),
        ("align = presteer_support(scene, recent, (steering, previous_steering)",
         "align = presteer_support(scene, frames, (steering, previous_steering)")):
    rep(old, new)
rep("""        recent = [scene]
        if self.execution.masked_history_support:
            try:
                recent, _ = self.recent.observe(scene, history)
            except MotionUnavailable:
                recent = [scene]
            recent = recent or [scene]""",
    """        frames = self.recent.update(scene, history) if self.execution.masked_history_support else []""")
left = [line for line in s.splitlines() if 'recent' in line.replace('self.recent', '').replace('recent_observation', '').replace('recent-observation', '')]
assert not left, left
with open(p, 'w', encoding='utf-8', newline='\n') as stream:
    stream.write(s)
print('patched')
