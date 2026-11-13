"""기본 미연결: 측방 띠/창의 깊이와 자체 가림 광선 탐색만 공유한다."""
from contextlib import contextmanager

import numpy as np

from arena_autonomy import pass_release_trial as ps


def band_check(scene, frame, sign, cfg, sel, longitudinal=None, *, verify=False, observer=None):
    """원 표본·띠/창·패딩·상태 판정 유지. 저장 결과/이전 입력은 재사용하지 않는다."""
    lo, hi = longitudinal if longitudinal is not None else (-cfg.length_m/2, cfg.length_m/2)
    xs = np.arange(lo, hi+1e-9, sel.band_step_m)
    ys = sign*np.arange(cfg.width_m/2+sel.band_offset_m, cfg.width_m/2+sel.band_width_m+1e-9, sel.band_step_m)
    q = np.stack(np.meshgrid(xs, ys, indexing='ij'), axis=-1).reshape(-1, 2)
    _, lateral = frame.project(q)
    q = q[np.abs(lateral) < cfg.wall_offset_m-cfg.wall_tolerance_m]
    if not len(q):
        return {'free': False, 'samples': 0, 'observed_free': 0, 'masked': 0, 'unobserved': 0}
    pad = ps.mx.ray_padding(scene, cfg, .5)
    classified = ps.depth_and_masked_support(scene, q, pad)
    free = ps.depth_support(scene, q, pad) if classified is None else classified[0]
    masked = np.zeros(len(q), dtype=bool)
    if np.any(~free):
        masked[~free] = ps.mx.masked_only(scene, q[~free], pad) if classified is None else classified[1][~free]
    if verify:
        original_free = ps.depth_support(scene, q, pad)
        original_masked = np.zeros(len(q), dtype=bool)
        if np.any(~original_free):
            original_masked[~original_free] = ps.mx.masked_only(scene, q[~original_free], pad)
        for label, actual, expected in (('free', free, original_free), ('masked', masked, original_masked)):
            if actual.dtype != expected.dtype or actual.shape != expected.shape or actual.tobytes() != expected.tobytes():
                raise AssertionError('r379 '+label+' full array mismatch')
    unobserved = ~free & ~masked
    result = {'free': bool(np.any(free)) and not bool(np.any(unobserved)), 'samples': int(len(q)),
              'observed_free': int(free.sum()), 'masked': int(masked.sum()), 'unobserved': int(unobserved.sum())}
    if observer is not None:
        observer(dict(shared=classified is not None, verified=verify, sign=sign,
                      longitudinal=longitudinal, band=result.copy()))
    return result


@contextmanager
def enabled(*, verify=False, observer=None):
    """현재 프로세스에 명시 연결하며 정상/예외 종료에 원 band_check를 복구한다."""
    original = ps.band_check
    def selected(scene, frame, sign, cfg, sel, longitudinal=None):
        return band_check(scene, frame, sign, cfg, sel, longitudinal, verify=verify, observer=observer)
    ps.band_check = selected
    try:
        yield
    finally:
        ps.band_check = original
