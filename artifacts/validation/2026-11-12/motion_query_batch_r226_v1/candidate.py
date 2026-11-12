"""원 steering 두 원호의 질의를 한 번에 생성하는 저장 입력 전용 후보."""
import numpy as np
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.point_set import unique_xy


def motion_queries(scene, speed, target, steering, previous, dt, follow_cfg, lat_cfg):
    age = scene.t_ref_s-scene.source_first_s
    distance = ps.mx.envelope_distance(max(abs(speed), abs(target)), age, dt, ps.mx.ExecutionConfig())
    poses, angles = [], []
    for angle in sorted({float(steering), float(previous)}):
        arc = ps.extrema_arc(angle, max(distance, .01), lat_cfg)
        poses.append(arc)
        angles.append(np.full(len(arc), angle))
    q, _ = ps.mx.body_queries(scene, np.vstack(poses), np.concatenate(angles), lat_cfg)
    q = unique_xy(q.reshape(-1, 2))
    outside = ((np.abs(q[:, 0]) > follow_cfg.length_m/2+1e-9)
               | (np.abs(q[:, 1]) > follow_cfg.width_m/2+1e-9))
    return distance, q[outside]
