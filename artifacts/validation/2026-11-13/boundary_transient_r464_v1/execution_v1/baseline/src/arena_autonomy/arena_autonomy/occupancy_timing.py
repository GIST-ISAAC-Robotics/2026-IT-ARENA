"""취득 시각별 점유 팽창. 점 제거/시간 갱신/상대 속도 이득은 하지 않는다."""
import math
import numpy as np


def per_ray_padding(scene, fixed_padding_m, motion_bound_mps):
    """보존된 반환점과 같은 순서의 축별 팽창량.

    기존 가정 |상대 물체 이동| <= bound * 취득 후 시간의 적용 시각만 바로잡는다.
    방사 오차/기본 여유와 물체 속도 상한은 그대로다. 이 상한의 실물 타당성,
    점 뒤 외곽이나 미래 운동까지 인증하지 않는다.
    """
    if any(isinstance(v, bool) or not math.isfinite(v) or v < 0
           for v in (fixed_padding_m, motion_bound_mps, scene.t_ref_s)):
        raise ValueError('invalid occupancy padding contract')
    rays = [ray for ray in scene.rays if ray.status == 'return']
    ages = np.asarray([scene.t_ref_s-ray.acquired_at_s for ray in rays], dtype=float)
    if np.any(~np.isfinite(ages)) or np.any(ages < 0):
        raise ValueError('invalid occupied point acquisition time')
    return fixed_padding_m+motion_bound_mps*ages
