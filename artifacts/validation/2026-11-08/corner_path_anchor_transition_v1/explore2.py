"""r22 시험의 원호 위 차량 코너 장면에서 바깥 면 관측 길이만 줄여 표현 전환 조건을 찾는다(탐색용)."""
import math, sys
import numpy as np
sys.path.insert(0, "src/arena_autonomy")
from arena_autonomy.local_path import local_path
O, R = .505, .347
def polyline(*v):
    return np.vstack([np.linspace(a, b, max(6, int(np.linalg.norm(np.array(b)-a)/.02)+1))[:-1] for a, b in zip(v, v[1:])]+[np.array(v[-1:])])
def corner_scan(mirror, degrees, lateral=0., face=3.5):
    wall = polyline([-1., -.505], [1.51, -.505], [1.51, -.505+face])*[1, mirror]
    a = math.radians(degrees)
    center = np.array([.5, .505*mirror]); radial = np.array([math.sin(a), -mirror*math.cos(a)])
    pose = center+(O+lateral)*radial; h = mirror*a
    rotation = np.array([[math.cos(h), math.sin(h)], [-math.sin(h), math.cos(h)]])
    return (wall-pose)@rotation.T
for mirror in (1, -1):
    side = 'right' if mirror == 1 else 'left'
    for deg in (20, 25, 30, 35, 40):
        for face in (3.5, .3, .2, .15, .1, .08, .05):
            for lat in (0, -.2, .2):
                p, i = local_path(corner_scan(mirror, deg, lat, face), side, O, piecewise_corner=True, minimum_radius=R)
                print(mirror, deg, face, lat, i.get('reason'), i.get('wall_model'),
                      None if p is None else round(i['wall_distance_m'], 3), None if p is None else round(i['path_anchor_error_m'], 4),
                      None if p is None else round(i['boundary_observed_ahead_m'], 2))
