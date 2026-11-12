def _baseline_command(self, scene, history, speed, side, previous_steering=0., control_dt=.02, *, allow_partial=False):
    cfg, now = self.config, scene.t_ref_s
    detail = {'mode': 'g1_follow', 'source_first_s': scene.source_first_s,
              'source_last_s': scene.source_last_s, 'source_epoch': scene.source_epoch,
              'reference_stamp_s': now, 'preferred_wall': side,
              'relative_velocity_credit': False, 'passing_enabled': False,
              'measured_speed_mps': speed, 'reverse_motion': math.isfinite(speed) and speed < 0.}
    if (side not in ('left', 'right') or not all(math.isfinite(v) for v in (speed, previous_steering, control_dt))
            or not 0 < control_dt <= .2):
        self._discard_path()
        return self._hold('follow_invalid_state', detail)
    # 마커의 추종 벽 전환은 관측 경로의 무효가 아니다. 새 벽 후보를 다음 스캔에서 같은 기준으로 고른다.
    side_changed = self.side != side and not (cfg.boundary_bridge and self.path is not None)
    if (self.epoch != scene.source_epoch or side_changed or
            (self.last_ref is not None and now < self.last_ref) or
            (self.last_source is not None and scene.source_first_s is not None and
             scene.source_first_s < self.last_source)):
        self._discard_path()
    self.epoch, self.side, self.last_ref = scene.source_epoch, side, now
    if scene.source_first_s is None or now-scene.source_first_s >= .3:
        self._discard_path()
        return self._hold('follow_scene_stale', detail)
    # 후방 사각은 직진 전체를 금지하지 않지만 전방 미관측은 현 보호와 같이 정지.
    front = [ray for ray in scene.rays if abs(math.atan2(ray.direction_xy[1], ray.direction_xy[0])) <= math.radians(100)]
    if not front or any(ray.status != 'return' for ray in front):
        from arena_autonomy.observation_support import side_missing_candidate
        if not (allow_partial or (cfg.recent_observation_enabled and side_missing_candidate(scene))):
            self._discard_path()
            return self._hold('follow_front_unobserved', detail)
        # 경로 계산만 허용한다. command()의 실제 관측 지지 없이는 양수 출력을 반환하지 않는다.
    points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
    if len(points) < 30:
        self._discard_path()
        return self._hold('follow_scan_invalid', detail)
    fresh = self.last_source is None or scene.source_first_s > self.last_source
    self.last_source = scene.source_first_s
    age = now-scene.source_first_s
    # 고속용도 관측 벽에서만 더 멀리 본다. 값은 탐색 길이이며 자유 공간이 아니다.
    cruise = cfg.open_route_speed_mps
    horizon = max(1.8, .06 + cfg.standstill_gap_m + cruise*(age+.10) + cruise**2/4)
    ray_index = None
    if cfg.ray_wall_continuity and fresh:
        # 경계 선택이 넘기는 부분집합도 같은 실제 점이므로 좌표로 원시 광선 번호를 찾는다.
        ray_index = dict(zip(map(tuple, points.tolist()),
                             (r.source_index for r in scene.rays if r.status == 'return')))
    bridge = {'narrow_occluder_bridge': True} if cfg.narrow_occluder_bridge else {}
    def observed_path(cloud, wall_side):
        if cfg.piecewise_wall_trial:
            path, info = local_path(cloud, wall_side, cfg.wall_offset_m, horizon=horizon,
                              rays=None if ray_index is None else [ray_index[tuple(p)] for p in cloud.tolist()],
                              piecewise_corner=True, minimum_radius=cfg.wheelbase_m/math.tan(cfg.max_steering_rad),
                              **bridge)
            if cfg.traffic_boundary_refresh_trial and info.get('wall_model') != 'piecewise_corner':
                from arena_autonomy.fragmented_wall_hypothesis import propose
                candidate, evidence = propose(cloud, wall_side, cfg.wall_offset_m,
                                              cfg.wheelbase_m/math.tan(cfg.max_steering_rad))
                if candidate is not None and (path is None or path_agrees(candidate,path,cfg.boundary_agreement_m)):
                    from arena_autonomy.local_path import _path_ahead, _origin_error
                    sign = 1 if wall_side == 'left' else -1
                    index = int(np.argmin(np.linalg.norm(cloud-[0.,sign*cfg.wall_offset_m],axis=1)))
                    return candidate, dict(reason='local_path',wall_model='observed_fragment_corner',
                        boundary_samples=evidence['points_used'],boundary_observed_ahead_m=_path_ahead(candidate),
                        wall_distance_m=abs(float(cloud[index,1])),path_anchor_error_m=_origin_error(candidate),
                        path_points=candidate.tolist(),fragment_hypothesis={k:v for k,v in evidence.items()
                        if k not in ('source_indices','occluder_indices')})
            return path,info
        if ray_index is not None:
            return local_path(cloud, wall_side, cfg.wall_offset_m, horizon=horizon,
                              rays=[ray_index[tuple(p)] for p in cloud.tolist()], **bridge)
        if horizon == 1.8:
            return local_path(cloud, wall_side, cfg.wall_offset_m, **bridge)
        return local_path(cloud, wall_side, cfg.wall_offset_m, horizon=horizon, **bridge)
    carried = None
    try:
        if self.path is not None and now-self.path_source < cfg.memory_ttl_s:
            carried = transport_points(self.path, history, self.path_ref, now)
            moved = np.linalg.norm(carried-self.path, axis=1)
            if cfg.boundary_bridge:
                # 기본 탐색 길이 밖 점의 회전 지렛대로 같은 운동을 큰 이동으로 세지 않는다.
                moved = moved[np.linalg.norm(self.path, axis=1) <= 1.8]
            if len(moved) and np.max(moved) > cfg.memory_motion_limit_m:
                carried = None
        if fresh:
            candidate, info = observed_path(points, side)
            preferred, preferred_info = candidate, info
            candidate_wall, corroborated = side, False
            def sufficient(path, details):
                return (path is not None and details['boundary_observed_ahead_m'] >= cfg.boundary_min_ahead_m
                        and anchor_error(details, cfg) <= cfg.boundary_anchor_error_m)

            def supported(reference, wall_side):
                # 기준 경로와 일치하는 실제 반환만으로 wall_side 벽 후보를 다시 만든다.
                # 그 후보 자체가 긴 경계·anchor 조건과 형상 일치를 모두 만족해야 한다.
                selected = boundary_support(points, reference, wall_side, cfg)
                path, details = observed_path(selected, wall_side)
                ok = sufficient(path, details) and path_agrees(path, reference, cfg.boundary_agreement_m)
                dropped = 0
                if not ok:
                    refined = drop_protrusions(selected, reference, wall_side, cfg)
                    if len(refined) < len(selected):
                        path, details = observed_path(refined, wall_side)
                        ok = sufficient(path, details) and path_agrees(path, reference, cfg.boundary_agreement_m)
                        dropped = len(selected)-len(refined)
                return path, details, ok, {'boundary_support_points': len(selected),
                                           'boundary_protrusions_removed': dropped}
            eligible = sufficient(candidate, info)
            detail['boundary_selection'] = 'unfiltered'
            if carried is not None:
                if eligible and cfg.solo_route_only and cfg.corridor_arc_agreement:
                    # 현재 충분히 관측된 긴 벽만 비교한다. 코너 출구의 같은 x좌표 때문에
                    # 이력이 거절되는 경우 기존 공통 호길이 비교를 재사용한다.
                    match = retained_boundary_agreement(candidate, carried, info, cfg)
                    eligible = match is not None
                    if match == 'shared_arc':
                        detail['boundary_history_agreement'] = 'current_corner_arc'
                else:
                    eligible = eligible and path_agrees(candidate, carried, cfg.boundary_agreement_m)
                if not eligible:
                    candidate, info, eligible, support = supported(carried, side)
                    detail.update(boundary_selection='observed_history_support', **support)
            other_side = 'right' if side == 'left' else 'left'
            opposite_observed = []

            def opposite_wall():
                if not opposite_observed:
                    opposite_observed.append(observed_path(points, other_side))
                return opposite_observed[0]
            if (cfg.junction_continuity and carried is not None
                    and self.path_wall == other_side):
                # 본선의 반대 벽을 이미 선택했다면 현재도 보이는 그 벽을 먼저 갱신한다.
                # 갈림길로 열린 선호 벽으로 매 스캔 되돌아가며 경로를 조금씩 꺾지 않는다.
                opposite, other = opposite_wall()
                if (sufficient(opposite, other) and path_agrees(opposite, carried, cfg.boundary_agreement_m)
                        and (not sufficient(preferred, preferred_info)
                             or not path_agrees_arc(preferred, opposite, cfg.boundary_agreement_m))):
                    candidate, info, candidate_wall, eligible = opposite, other, other_side, True
                    detail['boundary_selection'] = 'junction_retained_wall_refresh'
            if (not eligible and cfg.traffic_boundary_refresh_trial and carried is not None
                    and self.path_wall == other_side):
                opposite, other = opposite_wall()
                retained_match = (retained_boundary_agreement(opposite, carried, other, cfg)
                                  if sufficient(opposite, other) else None)
                if retained_match is not None:
                    candidate,info,candidate_wall,eligible = opposite,other,other_side,True
                    detail['boundary_selection']='current_retained_wall_refresh'
                    detail['retained_boundary_agreement']=retained_match
            if (not eligible and cfg.traffic_boundary_refresh_trial and carried is not None
                    and sufficient(preferred, preferred_info)
                    and retained_boundary_agreement(preferred, carried, preferred_info, cfg) == 'shared_arc'):
                # 기존 경계의 현재 관측 갱신을 먼저 시도한다. 실패한 경우에만 선호 벽의
                # 충분한 현재 직각 경로를 같은 유효 이력과 대조한다. 점유 검사는 이후 유지한다.
                candidate, info, candidate_wall, eligible = preferred, preferred_info, side, True
                detail['boundary_selection'] = 'current_corner_alternative_agreement'
                detail['current_corner_agreement'] = 'shared_arc'
            if not eligible and preferred is not None and anchor_error(preferred_info, cfg) <= cfg.boundary_anchor_error_m:
                # 급커브 안쪽 벽은 짧아질 수 있다. 반대 벽만 보고 전환하지 않고,
                # 같은 스캔의 실제 선호 벽 경로와 중심 형상이 일치할 때만 사용한다.
                # 채택할 반대 벽에는 원래의 긴 경계 조건을 그대로 요구한다.
                opposite, other = opposite_wall()
                if (sufficient(opposite, other) and path_agrees(opposite, preferred, cfg.boundary_agreement_m)
                        and (carried is None or path_agrees(opposite, carried, cfg.boundary_agreement_m))):
                    candidate, info, candidate_wall = opposite, other, other_side
                    eligible, corroborated = True, True
                    detail['boundary_selection'] = 'bilateral_confirmed_alternative'
            bilateral = corroborated
            if not eligible:
                # 선호 벽 앞의 얇은 물체가 연결을 끊었거나 그 물체가 기억 경로 범위 밖이면
                # 같은 스캔의 긴 반대 벽을 기준으로 선호 벽의 실제 반환을 다시 고른다.
                # 선호 벽 후보 자체가 긴 경계·앵커·형상 일치를 모두 만족해야 하는 양측
                # 확인이며, 기억이 있으면 그 기억과도 일치해야 한다.
                opposite, other = opposite_wall()
                if sufficient(opposite, other):
                    reselected, reselected_info, bilateral, support = supported(opposite, side)
                    bilateral = bilateral and (carried is None or
                                               path_agrees(reselected, carried, cfg.boundary_agreement_m))
                    if bilateral:
                        candidate, info, eligible = reselected, reselected_info, True
                        detail.update(boundary_selection='bilateral_reference_support', **support)
            detail['boundary_candidate_reason'] = info['reason']
            detail['boundary_candidate_ahead_m'] = info.get('boundary_observed_ahead_m')
            if info.get('narrow_occluders_bridged'):
                detail['narrow_occluders_bridged'] = info['narrow_occluders_bridged']
            detail['bilateral_alternative_confirmed'] = corroborated
            if eligible and carried is None and not bilateral:
                # 초기/만료 뒤 재획득에는 반대쪽의 독립된 긴 관측도 일치해야 한다.
                opposite, other = opposite_wall()
                eligible = sufficient(opposite, other) and path_agrees(candidate, opposite, cfg.boundary_agreement_m)
                if (not eligible and cfg.low_speed_arc_reacquire and abs(speed) <= .10
                        and sufficient(opposite, other)
                        and path_agrees_arc(candidate, opposite, cfg.boundary_agreement_m)):
                    # 측정 평균 속도가 낮을 때만 쓴다. 단일 축이라 실제 정지를 증명하지 않으며
                    # 이후 확인 횟수·점유·재개 여유·독립 보호는 기존 판정을 그대로 따른다.
                    eligible = True
                    detail['bilateral_reference'] = 'low_speed_arc'
                if not eligible:
                    # 반대 벽 앞의 얇은 물체도 같은 실제 반환 선택으로만 제외한다.
                    _, _, eligible, support = supported(candidate, other_side)
                    if eligible:
                        detail.update(bilateral_reference='opposite_support',
                                      opposite_support_points=support['boundary_support_points'],
                                      opposite_protrusions_removed=support['boundary_protrusions_removed'])
                if not eligible and cfg.standing_wall_reacquire and abs(speed) <= .10 and sufficient(candidate, info):
                    # 재개 직후 합류부처럼 반대쪽이 열리면 양측 일치가 영영 풀리지 않는다. 선호 벽의 긴 현재
                    # 관측만 쓰며 확인 횟수·점유·재개 여유·독립 보호는 그대로 둔다. 빈 공간을 경로로 만들지 않는다.
                    eligible = True
                    detail['bilateral_reference'] = 'standing_preferred_wall'
            if not eligible and cfg.boundary_evidence_mode in ('fragmented_candidate', 'corner_anchor_candidate', 'corner_distance_candidate'):
                # 긴 반대 벽은 그대로 관측되어야 한다. 선호 벽의 각 실제 조각이
                # 그 형상을 지지해야 하며 가림 간격은 길이/자유 공간 증거가 아니다.
                opposite, other = opposite_wall()
                if sufficient(opposite, other):
                    evidence = fragment_evidence(points, opposite, side, cfg)
                    agrees_memory = carried is None or path_agrees(opposite, carried, cfg.boundary_agreement_m)
                    detail['fragmented_boundary_evidence'] = {**evidence,
                        'fragments': [{k: v for k, v in part.items() if k != 'source_indices'}
                                      for part in evidence['fragments']]}
                    detail['fragmented_agrees_memory'] = bool(agrees_memory)
                    if evidence['supported'] and agrees_memory:
                        candidate, info, candidate_wall = opposite, other, other_side
                        eligible, bilateral = True, True
                        detail['boundary_selection'] = 'bilateral_observed_fragments'
                        detail['boundary_candidate_reason'] = info['reason']
                        detail['boundary_candidate_ahead_m'] = info.get('boundary_observed_ahead_m')
            if not eligible and cfg.boundary_evidence_mode == 'corner_anchor_candidate':
                # 직선/분리 경계 계약은 유지한다. 급커브에서만 현재 차체를
                # 가로지르는 실제 안쪽 벽과 긴 바깥 경로를 직접 대조한다.
                opposite, other = opposite_wall()
                if sufficient(opposite, other):
                    evidence = corner_anchor_evidence(points, opposite, side, cfg)
                    agrees_memory = carried is None or path_agrees(opposite, carried, cfg.boundary_agreement_m)
                    detail['corner_boundary_evidence'] = {k: v for k, v in evidence.items()
                                                          if k != 'source_indices'}
                    detail['corner_agrees_memory'] = bool(agrees_memory)
                    if evidence['supported'] and agrees_memory:
                        candidate, info, candidate_wall = opposite, other, other_side
                        eligible, bilateral = True, True
                        detail.update(boundary_selection='bilateral_observed_corner_anchor',
                                      boundary_candidate_reason=info['reason'],
                                      boundary_candidate_ahead_m=info.get('boundary_observed_ahead_m'))
            if not eligible and cfg.boundary_evidence_mode == 'corner_distance_candidate':
                opposite, other = opposite_wall()
                if sufficient(opposite, other):
                    evidence = corner_distance_evidence(points, opposite, side, cfg)
                    detail['corner_distance_evidence'] = {k: v for k, v in evidence.items()
                                                          if k != 'source_indices'}
                    retained = (opposite[:evidence['path_end_exclusive']]
                                if evidence['supported'] else None)
                    agrees_memory = (retained is not None and (carried is None or
                        path_agrees(retained, carried, cfg.boundary_agreement_m)))
                    detail['corner_distance_agrees_memory'] = bool(agrees_memory)
                    if evidence['supported'] and agrees_memory:
                        candidate, info, candidate_wall = retained, other, other_side
                        eligible, bilateral = True, True
                        detail.update(boundary_selection='bilateral_observed_corner_distance',
                                      boundary_candidate_reason=info['reason'],
                                      boundary_candidate_ahead_m=info.get('boundary_observed_ahead_m'))
            if not eligible and carried is not None:
                prefix, prefix_match = traffic_corner_prefix(preferred, carried, preferred_info, cfg)
                if prefix_match is not None:
                    # 미관측 연장 없이 현재 보이는 짧은 경로로 교체한다. 긴 벽의 초기
                    # 확보 기준은 유지하고, 현재 길이의 제동/조향/원시 점유를 다시 검사한다.
                    candidate, info, candidate_wall, eligible = prefix, preferred_info, side, True
                    detail['current_corner_observed_wall_ahead_m'] = preferred_info['boundary_observed_ahead_m']
                    detail['boundary_selection'] = 'current_corner_prefix_agreement'
                    detail['current_corner_prefix_agreement'] = prefix_match
            if not eligible and cfg.junction_continuity and carried is not None:
                # 짧은 선호 벽으로 경로 길이를 줄이기 전에, 승인 경로와 일치하는
                # 긴 현재 반대 벽을 쓴다. 점유·제동·독립 보호 입력은 그대로 둔다.
                opposite, other = opposite_wall()
                prefix_anchor = (retained_forward_prefix(opposite, carried, other, cfg)
                                 if self.path_wall == other_side else None)
                if ((sufficient(opposite, other) and path_agrees(opposite, carried, cfg.boundary_agreement_m))
                        or prefix_anchor is not None):
                    candidate, info, candidate_wall, eligible = opposite, other, other_side, True
                    detail.update(boundary_selection='junction_opposite_continuity',
                                  boundary_candidate_reason=info['reason'],
                                  boundary_candidate_ahead_m=info.get('boundary_observed_ahead_m'))
                    if prefix_anchor is not None:
                        detail['retained_forward_prefix'] = prefix_anchor
            if (not eligible and cfg.boundary_bridge
                    and carried is not None and preferred is not None
                    and anchor_error(preferred_info, cfg) <= cfg.boundary_anchor_error_m
                    and path_agrees(preferred, carried, cfg.boundary_agreement_m)):
                # 긴 경계 조건만 못 채운 현재 관측이 직전 경로와 겹친다. 더 짧은 현재 경로로 줄여 쓴다.
                candidate, info, eligible = preferred, preferred_info, True
                detail['boundary_selection'] = 'short_current_history_agreement'
            if (not eligible and cfg.traffic_boundary_refresh_trial and cfg.piecewise_wall_trial
                    and carried is not None and self.path_wall in ('left', 'right')):
                from arena_autonomy.corner_boundary_support import propose
                supported_corner, support_evidence = propose(points, carried, self.path_wall,
                                                              cfg, now-self.path_source, scene=scene)
                detail['current_corner_support'] = {k: v for k, v in support_evidence.items()
                    if not k.endswith('_indices')}
                far_support = support_evidence.get('far_occlusion_support')
                if far_support is not None:
                    detail['current_corner_support']['far_occlusion_support'] = {
                        k: v for k, v in far_support.items() if not k.endswith('_indices')}
                if supported_corner is not None:
                    candidate, candidate_wall, eligible = supported_corner, self.path_wall, True
                    # 두 관측 면 사이 교차점은 경로 가설이다. 미관측 길이를 관측 길이로 기록하지 않는다.
                    info = dict(reason='local_path', wall_model='current_two_face_corner',
                        boundary_observed_ahead_m=None,
                        path_anchor_error_m=support_evidence['path_anchor_error_m'])
                    detail.update(boundary_selection='current_two_face_corner_support',
                        boundary_candidate_reason=info['reason'], boundary_candidate_ahead_m=None,
                        corner_supported_path_ahead_m=support_evidence['path_ahead_m'])
            recent_confirmed = False
            if cfg.junction_continuity and carried is None:
                # 갈림길 열림으로 한 스캔의 선호 벽이 짧아져 기억이 막 만료된 경우, 그 승인 경로를
                # 운동 보정한 것과 가까운 전방에서 일치하는 현재 긴 관측(선호 벽 먼저)만 다시 받는다.
                # 기억으로 주행하지 않고 미관측을 채우지 않는다. 양측 확인 없는 첫 재획득을 줄이는 대체다.
                recent = None
                reference = getattr(self, 'recent_path', None)
                if (self.path is not None and self.path_source is not None
                        and cfg.memory_ttl_s <= now-self.path_source <= 2*cfg.memory_ttl_s
                        and (reference is None or self.path_source > reference[2])):
                    # 막 만료된 승인 경로는 아래 HOLD 이후에야 recent_path로 옮겨졌다.
                    # 주행 기억으로 되살리지 않고 이번의 충분한 현재 관측을 비교하는 데만
                    # 즉시 쓴다. 계산 간격이 200ms일 때 반복되는 한 주기 정지를 막는다.
                    reference = (self.path, self.path_ref, self.path_source)
                if reference is not None and 0 <= now-reference[2] <= 2*cfg.memory_ttl_s:
                    recent = transport_points(reference[0], history, reference[1], now)
                    # 창을 TTL의 두 배로 둔 만큼 운동 보정 이동 한도도 같은 비율로만 늘린다.
                    if np.max(np.linalg.norm(recent-reference[0], axis=1)) > 2*cfg.memory_motion_limit_m:
                        recent = None
                hint = getattr(self, '_expiry_route_reference', None)
                reference_wall = (hint[1] if hint is not None and reference is not None
                                  and hint[0] == reference[2] else None)
                if (recent is not None and reference_wall is not None
                        and candidate_wall != reference_wall
                        and not path_agrees_arc(candidate, recent, cfg.boundary_agreement_m)):
                    eligible = False
                    detail['expiry_route_switch_rejected'] = 'forward_path_diverges'
                if (recent is not None and eligible and sufficient(candidate, info)
                        and path_agrees(candidate, recent, cfg.boundary_agreement_m)):
                    recent_confirmed = True
                    detail['recent_boundary_confirmation'] = 'current_observation_after_memory_expiry'
                elif recent is not None and not eligible:
                    for path, details, wall_side in ((preferred, preferred_info, side), (*opposite_wall(), other_side)):
                        if (sufficient(path, details) and path_agrees(path, recent, cfg.boundary_agreement_m)
                                and (reference_wall is None or wall_side == reference_wall
                                     or path_agrees_arc(path, recent, cfg.boundary_agreement_m))):
                            candidate, info, candidate_wall = path, details, wall_side
                            eligible = recent_confirmed = True
                            detail.update(boundary_selection='junction_recent_path_agreement',
                                          boundary_candidate_reason=info['reason'],
                                          boundary_candidate_ahead_m=info.get('boundary_observed_ahead_m'))
                            break
            if (eligible and cfg.junction_continuity and carried is not None
                    and candidate_wall != self.path_wall
                    and not path_agrees_arc(candidate, carried, cfg.boundary_agreement_m)):
                # 가까운 접선은 같아도 약 1 m 안에서 갈라지는 다른 벽 경로는
                # 본선 갱신으로 받지 않는다. 한 스캔 누락이면 기존 TTL 경로만 남긴다.
                # 경로 끝 밖 외삽이나 TTL 연장은 하지 않는다.
                eligible = False
                detail['junction_wall_switch_rejected'] = 'forward_path_diverges'
            if eligible:
                confirmed = carried is not None or recent_confirmed
                if not confirmed:
                    old = (transport_points(self.pending, history, self.pending_ref, now)
                           if self.pending is not None and now-self.pending_ref <= cfg.memory_ttl_s else None)
                    self.pending_count = self.pending_count+1 if old is not None and path_agrees(candidate, old, cfg.boundary_agreement_m) else 1
                    self.pending, self.pending_ref = candidate.copy(), now
                    confirmed = self.pending_count >= cfg.confirmations
                if confirmed:
                    self.path, self.path_ref, self.path_source = candidate.copy(), now, scene.source_first_s
                    self.path_wall = candidate_wall
                    self.recent_path = None
                    carried = candidate
            else:
                self.pending = None
                self.pending_count = 0
            detail['fresh_boundary_accepted'] = bool(eligible and self.path_source == scene.source_first_s)
        if carried is None:
            if cfg.junction_continuity and self.path is not None:
                self.recent_path = (self.path, self.path_ref, self.path_source)
            self.path = None
            return self._hold('follow_boundary_unconfirmed', detail)
        memory_age = now-self.path_source
        # 단독 선택에서는 양측 근거로 이미 승인한 현재 반대 벽 경로도 쓴다.
        # 관측이 없는 경로/기억만 남은 경로/교통 모드의 속도 범위는 넓히지 않는다.
        # 구조 적응 경로에서는 이번 스캔의 설명된 구조 광선만 증속 전제에서 뺀다.
        # 그 밖의 결측이 있으면 기본 목표만 내어 v4 예산·감속 판단을 받는다(1.2 m/s
        # 초과 후보가 결측 판정을 정지로 바꾸지 않게). 아직 후보일 뿐이며 _partial_command가
        # 그림자/기억/예상 밖 결측을 검사한 후에만 발행한다. 독립 보호층도 같은 검사를 수행한다.
        explained = getattr(self, 'structure_explained_now', None)
        structure_candidate = bool(allow_partial and cfg.known_structure_policy == 'structure_aware_v1' and
                                   explained is not None and front and
                                   all(r.status == 'return' or r.source_index in explained for r in front))
        front_ready = bool(front and all(r.status == 'return' for r in front))
        open_route = bool(cruise and self.path_source == scene.source_first_s and
                          (self.path_wall == side or cfg.solo_route_only) and
                          (front_ready or structure_candidate))
        cap, desired, remaining = pursue_path(carried, speed, cfg, max(age, memory_age),
                                              speed_cap=cruise if open_route else None,
                                              **({'shape_diagnostic': detail} if cfg.solo_straight_budget_trial and open_route else {}))
        if cruise:
            detail.update(open_route_speed_enabled=open_route, open_route_cap_mps=cruise,
                          observed_search_horizon_m=horizon,
                          open_route_requires_structure_check=bool(open_route and not front_ready))
        if open_route and cfg.blend_open_speed_by_length:
            # 받은 경로의 길이가 줄면 통상 속도 위에 더한 부분만 걷는다.
            # 현재 경로/제동/곡률 한도보다 빠르게 하거나 빈 공간을 가정하지 않는다.
            share = float(np.clip((remaining - 1.10) / (1.70 - 1.10), 0., 1.))
            length_cap = cfg.max_speed_mps + share * (cruise - cfg.max_speed_mps)
            cap = min(cap, length_cap)
            detail['observed_length_speed_cap_mps'] = length_cap
        # WallFollow의 3 rad/s 명령 변화율을 반영한 실제 이번 발행각도 평가한다.
        steering = float(np.clip(desired, previous_steering-3*control_dt, previous_steering+3*control_dt))
        margin = .025 + cfg.measurement_padding_m + cfg.object_motion_bound_mps*age
        if cfg.occupancy_timing_mode == 'per_ray_candidate':
            margin = per_ray_padding(scene, .025+cfg.measurement_padding_m, cfg.object_motion_bound_mps)
            detail.update(occupancy_timing_mode=cfg.occupancy_timing_mode,
                occupancy_padding_min_m=float(np.min(margin)),
                occupancy_padding_max_m=float(np.max(margin)),
                occupancy_padding_points=len(margin))
        clearance = min(swept_limit(points, angle, cfg.wheelbase_m, horizon=max(2.5,horizon) if open_route else 2.5, margin=margin,
                                     length=cfg.length_m, width=cfg.width_m)
                        for angle in (steering, previous_steering))
        available = max(0., clearance-cfg.standstill_gap_m)
        target = min(cap, braking_speed(available, age), available/cfg.headway_s)
        detail.update(path_source_s=self.path_source, boundary_age_s=memory_age,
            boundary_from_memory=self.path_source != scene.source_first_s, path_wall=self.path_wall,
            wall_fallback=self.path_wall != side, path_points=carried.tolist(), observed_path_ahead_m=remaining,
            swept_clearance_m=clearance, occupancy_padding_m=float(np.max(margin)), occupied_points=len(points),
            unfiltered_occupancy=True, target_speed_before_hold_mps=target)
        if speed < 0.:
            # 유한한 signed 피드백은 입력 무효/시간 세대 변경이 아니다. 현재
            # 관측·signed 운동 보정·경계/점유 검사는 유지하되 전진은 금지한다.
            # 매 역방향 표본마다 재개 확인을 지워 지속 역행 중 주행하지 않으며,
            # 정상 측정 복귀 뒤에도 기존 서로 다른 두 fresh 관측을 요구한다.
            return self._hold('follow_reverse_motion', detail)
        if clearance <= cfg.hold_clearance_m:
            if (cfg.low_speed_arc_reacquire or
                (cfg.standing_wall_reacquire and self.path_source == scene.source_first_s)) and abs(speed) <= .10:
                # 속도 0을 유지한 채 조향만 경로 쪽으로 돌린다. 전진은 재개 여유와 독립 보호가 정한다.
                # 스캔 말미의 시간 여유 증가로 생긴 낮은 값이 새 스캔의 재개 계수를 지우지 않게 한다.
                self.held = True
                return 0., steering, {**detail, 'reason': 'follow_steer_in_place', 'behavior': 'HOLD'}
            return self._hold('follow_gap_hold', detail)
        if self.held:
            if clearance < cfg.resume_clearance_m:
                self.release_count = 0
            elif fresh:
                self.release_count += 1
            if self.release_count < cfg.confirmations:
                return 0., steering, {**detail, 'reason': 'follow_release_wait', 'behavior': 'HOLD'}
            self.held = False
        if (cfg.preview_turn_speed and open_route and 0. < target
                and (target <= cfg.max_speed_mps or cfg.preview_above_base_speed)
                and target < cap and clearance >= cfg.resume_clearance_m):
            from arena_autonomy.turn_preview import preview_speed
            before = target
            target, reason = preview_speed(scene, carried, cfg, speed, steering, previous_steering,
                                          age, cap, target, clearance, remaining)
            detail.update(turn_preview=reason, constant_arc_target_mps=before,
                          target_speed_before_hold_mps=target)
        behavior = 'FOLLOW' if target < cap-1e-6 else 'CRUISE'
        return float(target), steering, {**detail, 'reason': 'local_follow', 'behavior': behavior}
    except MotionUnavailable as error:
        self._discard_path()
        return self._hold('follow_'+str(error), detail)
