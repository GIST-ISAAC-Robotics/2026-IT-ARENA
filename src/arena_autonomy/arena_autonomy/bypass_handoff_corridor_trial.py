"""명시 시뮬레이션 자동 전환 후보. 다음 스캔 잔여 시간과 같은 통로 경로를 비교한다.
기본 registered_v0에는 사용하지 않는다. 센서 시각의 예측은 약속된 새 스캔/관측 허가가 아니다.
"""
from dataclasses import replace
import math
import numpy as np
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


def configure_guide(owner):
    old=owner.bypass.guide
    owner.bypass.guide=ForecastGuide(owner.config,old.registered_entry,
        stationary_outer_boundary=old.stationary_outer_boundary,wall_reanchor=old.wall_reanchor,
        tracked_wall_fit=old.tracked_wall_fit,rejoin_enabled=old.rejoin_enabled,
        corridor_policy=old.corridor_policy)

def discard_path_only(owner):
    # 운행 허가·경로·초기벽 확인은 삭제한다. 같은 시각축의 면 관측만 보관한다.
    # 실제 입력 고장/시간축 변경은 command 앞의 원래 reset 경로가 모두 지운다.
    guide=owner.bypass.guide
    monitors,history=guide.monitors,guide.motion_history
    guide.reset()
    guide.monitors,guide.motion_history=monitors,history

def execution_entries(owner,scene,speed,previous,dt):
    rows=owner.monitor.rows
    interval=rows[-1]['source']-rows[-2]['source'] if len(rows)>=2 else None
    if interval is None or not 0<interval<=.25:
        return [],dict(reason='scan_interval_unavailable')
    elapsed=scene.t_ref_s-rows[-1]['time']
    if not 0<=elapsed<=interval+1e-9:
        return [],dict(reason='observed_scan_horizon_expired',interval_s=interval,elapsed_s=elapsed)
    growth=max(0.,interval-elapsed)
    guide=owner.bypass.guide
    plan=guide._plan(scene)
    if plan is None or plan['chosen'] is None:
        return [],dict(reason='current_corridor_plan_unavailable',padding_growth_s=growth)
    row=plan['rows'][plan['chosen']]
    offset=guide.offset if guide.path is not None else row['offset_m']
    transition=guide.transition if guide.path is not None else row['transition_m']
    entry=replace(guide.registered_entry,offsets_m=(0.,offset),transitions_m=(transition,))
    result=_compare_entries(scene,scene.t_ref_s,speed,previous,dt,config=owner.config,
        entry=entry,stationary_boundaries=True,padding_growth_s=growth,padding_provider=guide._padding)
    # 재관측을 수반하는 지역 진행이다. 전체 전이의 자유 공간을 선언하지 않는다.
    # 미래 여유로 실제 상한 명령과 정지 영역까지 성립하는 후보만 받는다.
    usable=sorted((r['offset_m'],r['transition_m']) for r in result['candidates']
        if local_admission(r,owner.bypass.guide.entry.speed_cap_mps))
    return usable,dict(reason=result['reason'],padding_growth_s=growth,scan_interval_s=interval,
        since_first_receipt_s=elapsed,consistent=[list(u) for u in usable],
        candidates=[{k:r.get(k) for k in ('offset_m','transition_m','eligible','reason','preview_point_gap_m',
            'preview_return_progress_m','target_speed_mps','motion_region')} for r in result['candidates']],
        free_space_assumed=False,full_bypass_verified=False,
        policy='same_corridor_local_until_observed_next_scan',next_scan_arrival_guaranteed=False)
