import numpy as np
def sampled_region_rejection(scene, speed, target, steer, previous, dt, config, lateral, padding):
    """전체 검사와 같은 질의의 일부가 실패하면 즉시 거부한다. 통과 허가는 만들지 않는다.

    기존 영역 검사는 미관측 표본 하나로도 거부한다. 최대 16표본에서 이미 그 반례가
    있으면 수천 표본을 끝까지 분류할 필요가 없다. 표본 통과 시 전체 검사를 수행한다.
    """
    from arena_autonomy.pass_release_trial import motion_queries, region_blocked, SelectorConfig
    from arena_autonomy.observation_support import depth_support
    from arena_autonomy.occupancy_timing import per_ray_padding
    import arena_autonomy.m3_execution_trial as mx

    distance, queries = motion_queries(scene, speed, target, steer, previous, dt, config, lateral)
    if not len(queries):
        return None
    extent = np.maximum(np.abs(queries[:,1])-config.width_m/2,-queries[:,0]-config.length_m/2)
    # 현재 차체 뒤쪽에서 실제 가림 꼬리 기준을 넘는 표본을 먼저 검사한다.
    # 실제 전체 질의의 부분집합이며 거절만 앞당긴다.
    unsafe=np.flatnonzero((queries[:,0]<0.) & (extent>SelectorConfig().masked_tail_allowance_m))
    if not len(unsafe):return None
    q=queries[unsafe[np.argsort(extent[unsafe])[-16:]]]

    pad = mx.ray_padding(scene, lateral, config.object_motion_bound_mps)
    base_free = depth_support(scene, q, pad)
    if np.all(base_free):
        return None
    # 실제 전체 검사도 기본 여유에서 미지지 표본이 있을 때만 벽 여유를 재분류한다.
    # 이 부분집합의 미지지 표본이 그 분기 진입을 증명한다.
    if padding is not None:
        returns = np.asarray([r.status == 'return' for r in scene.rays])
        original = per_ray_padding(scene, .025+config.measurement_padding_m, config.object_motion_bound_mps)
        fixed = padding < original
        pad[np.flatnonzero(returns)[fixed]] = np.maximum(.025+lateral.budget.range_bias_m, padding[fixed])
    free = depth_support(scene, q, pad) if padding is not None else base_free
    masked = np.zeros(len(q), dtype=bool)
    if np.any(~free):
        masked[~free] = mx.masked_only(scene, q[~free], pad)
    tail = q[masked]
    region = dict(distance_m=float(distance), queries=len(queries),
        unobserved=int(np.sum(~free & ~masked)),
        masked_left=int(np.sum(masked & (q[:, 1] > 0))),
        masked_right=int(np.sum(masked & (q[:, 1] < 0))),
        masked_front_samples=int(np.sum(tail[:, 0] > config.length_m/2)) if len(tail) else 0,
        masked_tail_extent_m=float(np.max(np.maximum(abs(tail[:, 1])-config.width_m/2,
            -tail[:, 0]-config.length_m/2))) if len(tail) else 0.,
        sampled_rejection_only=True, checked_samples=len(q), counts_are_lower_bounds=True)
    return region if region_blocked(region, SelectorConfig()) else None
