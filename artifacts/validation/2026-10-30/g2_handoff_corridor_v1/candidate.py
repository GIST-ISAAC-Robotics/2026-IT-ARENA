"""오프라인 자동 전환 후보. 다음 스캔 잔여 시간과 같은 통로 경로를 비교한다.
운영 코드/기본값 미변경. 센서 시각의 예측은 약속된 새 스캔/관측 허가가 아니다.
"""
from dataclasses import replace
import math
import numpy as np
from arena_autonomy.bypass_handoff import BypassHandoffFollow
from arena_autonomy.bypass_entry_trial import _compare_entries, current_outer_wall_padding
from arena_autonomy.bypass_signed_hold_trial import SignedHoldGuide
from arena_autonomy.bypass_corridor_trial import wall_frame, boundary_face, connected_wall_returns
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.observation_support import source_valid
from arena_autonomy.scene_motion import transport_points


def local_admission(row,cap):
    # eligible은 기존 현재 점유/관측/제동 영역 검사의 결과다. 여기서는 미래
    # 광선 나이 여유를 적용한 같은 경로가 실제 명령 상한까지 지지하는지 더 본다.
    target=row.get('target_speed_mps')
    return (row.get('eligible') is True and bool(row.get('offset_m'))
        and isinstance(target,(int,float)) and not isinstance(target,bool)
        and math.isfinite(target) and target+1e-12>=cap)

class ForecastGuide(SignedHoldGuide):
    def evaluate(self,scene,history,speed,previous,dt):
        original_entry,original_registered=self.entry,self.registered_entry
        primary=super().evaluate(scene,history,speed,previous,dt)
        if (self.path is None or primary.get('reason')!='no_entry_command_candidate'
                or primary.get('selected') is not None or speed<0):
            return primary
        # 이미 관측해 기억하는 경로에서만 저속 명령을 새로 계산한다.
        # measured speed/직전 조향/원시 반환/마스크는 그대로다. 재사용한 같은
        # 스캔은 새 관측 확인 횟수가 아니며 signed recovery 대기를 우회하지 않는다.
        attempts=[]
        try:
            for cap in (.25,.15,.08):
                if cap>=original_entry.speed_cap_mps:continue
                self.entry=replace(original_entry,speed_cap_mps=cap)
                self.registered_entry=replace(original_registered,speed_cap_mps=cap)
                alternative=super().evaluate(scene,history,speed,previous,dt)
                selected=alternative.get('selected')
                attempts.append(dict(cap_mps=cap,reason=alternative['reason'],selected=selected is not None))
                if selected is not None:
                    return dict(alternative,slow_command_trial=dict(applied=True,
                        primary_reason=primary['reason'],attempts=attempts,measurement_unchanged=True))
                if self.path is None:break
            return dict(primary,slow_command_trial=dict(applied=False,primary_reason=primary['reason'],
                attempts=attempts,measurement_unchanged=True))
        finally:
            self.entry,self.registered_entry=original_entry,original_registered

    def _padding(self, scene, config, side, wall, growth_s=0.):
        padding, evidence = current_outer_wall_padding(scene, config, side, wall, growth_s)
        if not math.isfinite(growth_s) or not 0. <= growth_s <= .25:
            raise ValueError('invalid observed scan horizon')
        p = self.corridor_policy
        padding = np.asarray(padding, dtype=float).copy()
        before = padding.copy()
        points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
        rays = [ray for ray in scene.rays if ray.status == 'return']
        ids = np.asarray([ray.source_index for ray in rays])
        fixed = .025+config.measurement_padding_m
        static = padding <= fixed+1e-12
        along, normal = wall_frame(points, config, side, wall)
        if wall.get('reason') != 'current_connected_straight_wall_fit':
            static = static|connected_wall_returns(ids, points, along, normal, p)
            padding[static] = np.minimum(padding[static], fixed)
        face = boundary_face(ids, points, along, normal, ~static, p)

        def to_current(row):
            if self.motion_history is None or row['endpoints'] is None:
                return None
            try:
                moved = transport_points(row['endpoints'], self.motion_history, row['reference'], scene.t_ref_s)
            except (ValueError, MotionUnavailable):
                return None
            values = wall_frame(moved, config, side, wall)[0]
            return float(values.min()), float(values.max())
        if face is not None:
            members = face['members']
            face['endpoints_xy_m'] = points[[members[int(np.argmin(along[members]))], members[int(np.argmax(along[members]))]]]
        state = self.monitors[side].update(scene, face, to_current)
        use = dict(side=side, surface=None if face is None else dict(normal_m=face['normal_m'], method=face['method'],
                low_order_m=face['low_order_m'], bins=face['bins'], along_range_m=list(face['along_range_m'])),
            state=state, wall_rays_aligned=0, inner_rays_aligned=0, onset_rays=0, supported_rays=0, supported_along_m=None)
        supported = np.zeros(len(points), dtype=bool)
        if face is not None:
            # 근거는 이 면 구간에만 적용한다. 표면 띠 밖 반환은 면 구간 안이어도 그대로 둔다.
            supported[face['members']] = True
            supported &= np.abs(normal-face['normal_m']) <= p.snap_budget_m
            if supported.any():
                use.update(supported_rays=int(supported.sum()),
                           supported_along_m=[float(along[supported].min()), float(along[supported].max())])
        if state.get('stable') and supported.any():
            ages = np.asarray([scene.t_ref_s-ray.acquired_at_s+growth_s for ray in rays], dtype=float)
            reduced = fixed+state['observed_rate_term_mps']*ages+.5*p.onset_acceleration_mps2*ages*ages
            padding[supported] = np.minimum(padding[supported], reduced[supported])
            use['onset_rays'] = int(supported.sum())
        wall_snap = np.clip(normal, 0., p.snap_budget_m)*static
        padding -= wall_snap
        use['wall_rays_aligned'] = int(np.count_nonzero(wall_snap))
        if face is not None:
            inner_snap = np.clip(face['normal_m']-normal, 0., p.snap_budget_m)*supported
            padding -= inner_snap
            use['inner_rays_aligned'] = int(np.count_nonzero(inner_snap))
        use['maximum_inner_padding_m'] = float(np.max(padding[supported])) if supported.any() else None
        use['padding_never_increased'] = bool(np.all(padding <= before+1e-12))
        self.corridor_use[side] = use
        return padding, dict(evidence, corridor=use)


class CorridorHandoff(BypassHandoffFollow):
    def __init__(self,*args,**kw):
        super().__init__(*args,**kw)
        if (self.execution.bypass.corridor_policy!='corridor_v2_signed_hold'
                or self.execution.preparation_policy!='observed_entry_space_v1'
                or self.execution.persistence_window_policy!='trim_before_compare_v1'):
            raise ValueError('explicit corridor and space policy required')
        old=self.bypass.guide
        self.bypass.guide=ForecastGuide(self.config,old.registered_entry,
            stationary_outer_boundary=old.stationary_outer_boundary,wall_reanchor=old.wall_reanchor,
            tracked_wall_fit=old.tracked_wall_fit,rejoin_enabled=old.rejoin_enabled,
            corridor_policy=old.corridor_policy)

    def _discard_path_only(self):
        # 운행 허가·경로·초기벽 확인은 삭제한다. 같은 시각축의 면 관측만 보관한다.
        # 실제 입력 고장/시간축 변경은 command 앞의 원래 reset 경로가 모두 지운다.
        guide=self.bypass.guide
        monitors,history=guide.monitors,guide.motion_history
        guide.reset()
        guide.monitors,guide.motion_history=monitors,history

    def _execution_entries(self,scene,speed,previous,dt):
        rows=self.monitor.rows
        interval=rows[-1]['source']-rows[-2]['source'] if len(rows)>=2 else None
        if interval is None or not 0<interval<=.25:
            return [],dict(reason='scan_interval_unavailable')
        elapsed=scene.t_ref_s-rows[-1]['time']
        if not 0<=elapsed<=interval+1e-9:
            return [],dict(reason='observed_scan_horizon_expired',interval_s=interval,elapsed_s=elapsed)
        growth=max(0.,interval-elapsed)
        guide=self.bypass.guide
        plan=guide._plan(scene)
        if plan is None or plan['chosen'] is None:
            return [],dict(reason='current_corridor_plan_unavailable',padding_growth_s=growth)
        row=plan['rows'][plan['chosen']]
        offset=guide.offset if guide.path is not None else row['offset_m']
        transition=guide.transition if guide.path is not None else row['transition_m']
        entry=replace(guide.registered_entry,offsets_m=(0.,offset),transitions_m=(transition,))
        result=_compare_entries(scene,scene.t_ref_s,speed,previous,dt,config=self.config,
            entry=entry,stationary_boundaries=True,padding_growth_s=growth,padding_provider=guide._padding)
        # 재관측을 수반하는 지역 진행이다. 전체 전이의 자유 공간을 선언하지 않는다.
        # 미래 여유로 실제 상한 명령과 정지 영역까지 성립하는 후보만 받는다.
        usable=sorted((r['offset_m'],r['transition_m']) for r in result['candidates']
            if local_admission(r,self.bypass.guide.entry.speed_cap_mps))
        return usable,dict(reason=result['reason'],padding_growth_s=growth,scan_interval_s=interval,
            since_first_receipt_s=elapsed,consistent=[list(u) for u in usable],
            candidates=[{k:r.get(k) for k in ('offset_m','transition_m','eligible','reason','preview_point_gap_m',
                'preview_return_progress_m','target_speed_mps','motion_region')} for r in result['candidates']],
            free_space_assumed=False,full_bypass_verified=False,
            policy='same_corridor_local_until_observed_next_scan',next_scan_arrival_guaranteed=False)

    def command(self, scene, history, speed, side, previous, dt, *, raw_scan=None):
        if (raw_scan is None or not all(isinstance(v, (int,float)) and not isinstance(v,bool)
                and math.isfinite(v) for v in (speed,previous,dt))
                or abs(previous) > self.config.max_steering_rad or not 0 < dt <= .2
                or side not in ('left','right') or not source_valid(scene)):
            self.reset()
            return self._output(0., 0., {}, 'handoff_invalid_input')
        key = (scene.source_frame_id, scene.source_epoch, scene.source_first_s)
        if self.last_key is not None and (key[:2] != self.last_key[:2]
                or key[2] is None or self.last_key[2] is None or key[2] < self.last_key[2]
                or scene.t_ref_s < self.last_ref):
            self.reset()
            return self._output(0., previous, {}, 'handoff_source_reset')
        # G1의 옆차/소실 이력은 G2 중에도 관측한다. 모드 전환만으로 지우지 않는다.
        self.bypass.guide.motion_history = history
        normal = self.normal.command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
        persistence = self.monitor.update(scene, history)
        fresh = key != self.last_key
        self.last_key = key; self.last_ref = scene.t_ref_s
        detail = dict(mode_before=self.mode, normal=normal[2], persistence=persistence,
                      scope='explicit_simulation_handoff_trial')
        fault = normal[2].get('selector_choice') == 'hold_fault'
        if fault or not source_valid(scene) or not math.isfinite(speed) or (speed < 0 and self.mode == 'FOLLOW'):
            self.bypass.reset(); self.monitor.reset(); self.mode = 'FOLLOW'; self.reacquire = 0
            # 유한한 음의 엔코더 속도는 기존 G1이 정지하면서 관측/옆차 이력을
            # 보존하는 입력이다. 새 우회 경로만 버리고 G1 전체를 reset하지 않는다.
            return self._output(0., previous, detail,
                'handoff_reverse_motion_hold' if speed < 0 else 'handoff_input_fault')
        if self.mode == 'FOLLOW':
            # 확실한 현재 전방 반환을 보면서 관측 시간을 확보한다. 반환이 없어지면
            # G1을 따르며, 무반환 자체를 장애물/우회 신호로 사용하지 않는다.
            if self.execution.preparation_policy == 'front_patch_v0':
                prepare = persistence['observed']
            else:
                detail['preparation'] = self._preparation(scene, history, normal, persistence, previous,
                    require_above_cap=self.execution.preparation_policy != 'observed_entry_space_v1')
                prepare = detail['preparation']['applied']
            # 준비 감속 여부와 무관하게 낮은 이동 누적과 아래 진입 조건은 같다.
            target = min(normal[0], self.execution.preparation_cap_mps) if prepare else normal[0]
            if self.execution.follow_entry_space_enabled:
                target, detail['follow_entry_space'] = self._follow_entry_space(scene, normal, previous, target)
            space = self.execution.preparation_policy == 'observed_entry_space_v1'
            if space and prepare and not persistence['ready']:
                usable, detail['entry_space'] = self._execution_entries(scene, speed, previous, dt)
                longest = max(self.bypass.guide.entry.transitions_m)
                # 가장 완만한 진입이 아직 가능하면 더 다가가지 않는다. 이미 없으면 기다려도
                # 공간이 돌아오지 않으므로 G1을 따른다(아래 진입 검사는 그대로).
                if any(t == longest for _, t in usable):
                    target = 0.
                    detail['entry_space']['hold'] = True
                else:
                    detail['entry_space']['hold'] = False
            # .35는 출력 상한이다. 28카운트/15:1의 20 ms 속도 양자화로 측정값은
            # .340/.365를 오가므로 이를 입력 허용선으로 쓰면 관측을 매번 버린다.
            # 측정 속도는 그대로 G2에 전달하여 기존 반응/제동 영역에서 검사한다.
            if (persistence['ready'] and speed <= self.bypass.config.max_speed_mps
                    and abs(previous) <= self.execution.maximum_initial_steering_rad):
                candidate = self.bypass.command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
                chosen = candidate[2].get('selected')
                detail['bypass'] = candidate[2]
                if chosen is not None and chosen['offset_m'] != 0. and candidate[0] > 0 and space:
                    # 진입 순간의 여유로만 고른 경로가 다음 새 스캔 전에 막히면 반쯤 들어가 멈춘다.
                    usable, detail['entry_admission'] = self._execution_entries(scene, speed, previous, dt)
                    if (chosen['offset_m'], chosen['transition_m']) not in usable:
                        self._discard_path_only()
                        detail['entry_admission']['admitted'] = False
                        return self._output(target, normal[1], detail, 'handoff_follow')
                    detail['entry_admission']['admitted'] = True
                if chosen is not None and chosen['offset_m'] != 0. and candidate[0] > 0:
                    selected_side = 'left' if chosen['offset_m'] > 0 else 'right'
                    threat = normal[2].get('selector_sides', {}).get(selected_side)
                    # 미관측 뒤차 부재 증명은 아니며 기존 소실/측방 위협을 보존한다.
                    if threat is not None and not threat.get('present') and not threat.get('seen'):
                        self.mode = 'BYPASS'
                        return self._output(candidate[0], candidate[1], detail, 'handoff_entry')
                    self.bypass.reset()
            else:
                self._discard_path_only()
            return self._output(target, normal[1], detail, 'handoff_follow')
        candidate = self.bypass.command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
        detail['bypass'] = candidate[2]
        centered = candidate[2].get('entry_phase') == 'centered'
        if centered:
            self.mode = 'REACQUIRE'
            # 경로/점유 검사를 완료한 G1의 간격 정지 역시 정상 출력이다. 이전의
            # 양수만 허용하는 연결은 이미 중앙에 복귀했어도 G2로 계속 접근했다.
            # 명시 후보에서만 알려진 정지 이유와 경로 근거를 인정하며 고장/경계
            # 미확정을 정상 준비로 바꾸지 않는다. 서로 다른 두 스캔은 그대로다.
            nd = normal[2]
            known_hold = (self.execution.reacquire_policy == 'known_hold_v1' and normal[0] == 0.
                and nd.get('reason') in ('follow_gap_hold','follow_release_wait')
                and isinstance(nd.get('path_points'),list) and len(nd['path_points']) >= 2
                and isinstance(nd.get('observed_path_ahead_m'),(int,float)) and nd['observed_path_ahead_m'] > 0
                and isinstance(nd.get('swept_clearance_m'),(int,float))
                and math.isfinite(nd['swept_clearance_m']) and nd['swept_clearance_m'] >= 0)
            ready = normal[0] > 0 or known_hold
            self.reacquire = self.reacquire + 1 if fresh and ready else self.reacquire
            if not ready:
                self.reacquire = 0
            if self.reacquire >= self.execution.reacquire_distinct_scans:
                self.mode = 'FOLLOW'; self.bypass.reset(); self.monitor.reset(); self.reacquire = 0
                return self._output(min(normal[0], self.execution.preparation_cap_mps), normal[1], detail, 'handoff_return')
            if known_hold:
                return self._output(0., normal[1], detail, 'handoff_reacquire_hold')
        else:
            self.reacquire = 0
            self.mode = 'BYPASS'
        return self._output(candidate[0], candidate[1], detail, 'handoff_bypass')
