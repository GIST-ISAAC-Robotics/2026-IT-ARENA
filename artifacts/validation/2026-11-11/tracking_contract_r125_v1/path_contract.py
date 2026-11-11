"""r117의 경로 관측 수명 수정. 연구 전용이며 운영 허가를 추가하지 않는다."""
from dataclasses import dataclass, replace
import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_follow import pursue_path
from arena_autonomy.scene_motion import transport_points

_path = Path(__file__).resolve().parents[1]/'current_tracking_r117_v1/candidate.py'
_spec = importlib.util.spec_from_file_location('tracking_r117_base', _path)
base = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = base
_spec.loader.exec_module(base)


@dataclass(frozen=True)
class PathProposal(base.PathProposal):
    path_source_s: float


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def proposal(data, output):
    plan = base.proposal(data, output)
    if plan is None:
        return None
    source = output[2]['normal'].get('path_source_s')
    # 재계산 시각이나 최신 스캔 시각으로 옛 경로의 원 관측시각을 대체하지 않는다.
    if not finite(source) or not 0 <= source <= plan.source_first_s <= plan.reference_s:
        return None
    return PathProposal(**vars(plan), path_source_s=source)


def current_candidate(plan, data, scan, history, scene, config, *, output_dt_s=.02):
    def reject(reason, **extra):
        return dict(eligible=False, reason=reason, **extra)

    if plan is None:
        return reject('no_positive_follow_plan')
    now = data['reference_s']
    source = plan.path_source_s
    if (not all(finite(v) for v in (now, source, plan.reference_s, plan.source_first_s))
            or not 0 <= source <= plan.source_first_s <= plan.reference_s <= now):
        return reject('plan_path_time_invalid')
    if now-source >= config.memory_ttl_s:
        return reject('plan_path_source_expired', path_source_s=source,
                      path_age_s=now-source, path_deadline_s=source+config.memory_ttl_s)
    if (data['epoch'] != plan.motion_epoch or scan.source_epoch != plan.sensor_epoch
            or data['side'] != plan.side):
        return reject('plan_epoch_or_side_changed')
    try:
        path = transport_points(plan.path, history, plan.reference_s, now)
        movement = np.linalg.norm(path-np.asarray(plan.path),axis=1)
        if len(movement) and np.max(movement)>config.memory_motion_limit_m:
            return reject('plan_path_motion_exceeded')
        # 경로 끝까지 제동하는 속도 계산도 실제 원 관측 나이를 사용한다.
        age = max(now-scene.source_first_s, now-source)
        cap, _, _ = pursue_path(path, abs(data['speed_mps']), config, age,
                               speed_cap=plan.speed_cap_mps)
        bounded = replace(plan, speed_cap_mps=min(plan.speed_cap_mps, cap))
        result = base.current_candidate(bounded, data, scan, history, scene, config,
                                        output_dt_s=output_dt_s)
        return {**result, 'path_source_s':source, 'path_age_s':now-source,
                'path_deadline_s':source+config.memory_ttl_s,
                'plan_original_speed_cap_mps':plan.speed_cap_mps,
                'runtime_authorized':False}
    except (MotionUnavailable, ValueError) as error:
        return reject('current_geometry_unavailable', error=str(error))
