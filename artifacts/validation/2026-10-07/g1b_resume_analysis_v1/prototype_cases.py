"""탐색용: 합성 기하 반례의 매개값을 정하기 위한 시험 전 확인(제어 변경 없음)."""
import math, sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'tests')]
from test_local_follow import history  # noqa
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.local_follow import LocalFollow, boundary_support
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import transport_points


def box(cx, cy, sx, sy):
    x0, x1, y0, y1 = cx-sx/2, cx+sx/2, cy-sy/2, cy+sy/2
    c = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [(c[i], c[(i+1) % 4]) for i in range(4)]


def corridor(left=.425, right=-.425, start=-3., end=4.):
    return [((start, left), (end, left)), ((start, right), (end, right)), ((end, left), (end, right))]


def cast_scene(now, segments, source=None, epoch=0, samples=500):
    seg = np.asarray(segments, float)
    p, q = seg[:, 0], seg[:, 1]-seg[:, 0]
    w = p-np.array([.06, 0.])
    angles = -math.pi+np.arange(samples)*math.tau/samples
    d = np.column_stack((np.cos(angles), np.sin(angles)))
    denom = d[:, None, 0]*q[None, :, 1]-d[:, None, 1]*q[None, :, 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (w[None, :, 0]*q[None, :, 1]-w[None, :, 1]*q[None, :, 0])/denom
        u = (w[None, :, 0]*d[:, None, 1]-w[None, :, 1]*d[:, None, 0])/denom
    hit = (np.abs(denom) > 1e-12) & (t > .05) & (u >= 0) & (u <= 1)
    ranges = np.where(hit, t, np.inf).min(axis=1)
    first = now-.1 if source is None else source
    return scene_from_scan(ranges.tolist(), angle_min=-math.pi, angle_increment=math.tau/samples, range_min=.05,
                           range_max=12., acquired_at_s=[first]*samples, t_ref_s=now, source_epoch=epoch)


def show(name, world, memory=True, now_list=(.8,), side='left'):
    ctl, h = LocalFollow(), history()
    if memory:
        for now in (.4, .5, .6, .7):
            ctl.command(cast_scene(now, corridor()), h, 0., side)
    for now in now_list:
        sc = cast_scene(now, world)
        pts = np.asarray(sc.occupied_endpoints_xy_m)
        L, li = local_path(pts, 'left'); R, ri = local_path(pts, 'right')
        extra = ''
        if ctl.path is not None:
            carried = transport_points(ctl.path, h, ctl.path_ref, now)
            sel = boundary_support(pts, carried, side, ctl.config)
            _, si = local_path(sel, side)
            extra = f"pass1 {si.get('reason')} {si.get('boundary_observed_ahead_m')}"
        sp, _, d = ctl.command(sc, h, 0., side)
        print(name, now, d['reason'], d.get('boundary_selection'), d.get('boundary_candidate_ahead_m'), d.get('fresh_boundary_accepted'),
              d.get('boundary_protrusions_removed'), d.get('bilateral_reference'), d.get('path_wall'), 'L', li.get('boundary_observed_ahead_m'), 'R', ri.get('boundary_observed_ahead_m'), extra, 'cmd', sp)


print('--- branch right')
for div in (.3, .5):
    branch = [((-3, .425), (4, .425)), ((-3, -.425), (div, -.425)), ((div, -.425), (2., -1.2)), ((2., -1.2), (4, -1.5)), ((4, .425), (4, -1.5))]
    show(f'branch {div} cold', branch+box(.36, .425-.07-.0125, .025, .025), memory=False, now_list=(.4, .5, .6, .7))
    show(f'branch {div} cold nopost', branch, memory=False, now_list=(.4, .5, .6, .7))
