"""측방 요약에서 사용하지 않는 nearest 격자만 생략한 동등 계산 후보."""
import numpy as np
import arena_autonomy.pass_release_trial as ps


def threat_view(scene, path_points, cfg):
    path = np.asarray(path_points if path_points is not None else [], dtype=float).reshape(-1, 2)
    if len(path) < 2 or not np.all(np.isfinite(path)):
        return None
    try:
        frame = ps.mx.ExecutionFrame(path)
    except (ValueError, TypeError):
        return None
    state = ps.mx.ego_state(frame, cfg)
    grid = ps.mx.ReturnGrid(scene, cfg)
    ds, masks = ps.mx.side_return_masks(frame, state, grid, cfg)
    out = {'frame': frame, 'state': state}
    for (name, _), mask in zip(ps.SIDES, masks):
        values = ds[mask] if len(ds) else np.zeros(0)
        count = int(np.count_nonzero(mask))
        out[name] = dict(present=count >= cfg.min_threat_points, points=count,
                         rearmost_ds_m=float(np.min(values)) if len(values) else None,
                         frontmost_ds_m=float(np.max(values)) if len(values) else None)
    return out


def install():
    ps.threat_view = threat_view
