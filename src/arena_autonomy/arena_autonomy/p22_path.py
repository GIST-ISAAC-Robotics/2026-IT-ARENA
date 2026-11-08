"""P22 센서 간극에 맞춘 짧은 우회전 경로. 지도/정답/주행 허가 없음."""
from dataclasses import dataclass, astuple
import math

WB, RATE, DS = .135, 3., .01
angle = lambda a: math.atan2(math.sin(a), math.cos(a))
dot = lambda x,y,v: x*v[0]+y*v[1]

@dataclass(frozen=True)
class Pose:
    x: float = -WB/2   # 현재 차체 좌표에서의 후륜 중심
    y: float = 0.
    yaw: float = 0.
    steer: float = 0.
    s: float = 0.

def segment(path, length, last_steer):
    p = path[-1]
    if length < -1e-8:
        raise ValueError('negative segment')
    if length <= 1e-10:
        assert abs(p.steer-last_steer) < 1e-8
        return
    n = math.ceil(length / DS)
    ds = length/n
    start = p.steer
    for i in range(1, n+1):
        end = start+(last_steer-start)*i/n
        dyaw = ds*math.tan((p.steer+end)/2)/WB
        half = dyaw/2
        chord = ds*(math.sin(half)/half if abs(half)>1e-9 else 1.)
        p = Pose(p.x+chord*math.cos(p.yaw+half), p.y+chord*math.sin(p.yaw+half),
                 p.yaw+dyaw, end, p.s+ds)
        path.append(p)

def curve(path, sr, steer, target, speed):
    ramp_len = abs(steer)*speed/RATE
    ramp = [Pose()]
    segment(ramp, ramp_len, steer)
    segment(ramp, ramp_len, 0.)
    plateau = (target-sr-ramp[-1].yaw)*WB/math.tan(steer)
    if plateau < 0:
        return False
    segment(path, ramp_len, steer)
    segment(path, plateau, steer)
    segment(path, ramp_len, 0.)
    return True

def turn(hit, previous, radius, speed, *, final_depth_m=.25):
    if not math.isfinite(final_depth_m) or not .25 <= final_depth_m <= .55:
        raise ValueError('limited P22 rear depth must be within .25..55 m')
    ax = math.radians(hit['a_heading_deg'])
    v = (math.cos(ax), math.sin(ax))
    n = (v[1], -v[0])
    g = [(a+b)/2 for a,b in zip(hit['e1'],hit['e2'])]
    target = ax-math.pi/2
    steer = -math.atan(WB/radius)
    assert abs(steer)<.37 and abs(previous)<=.37+1e-8 and speed>0
    approach = [Pose(steer=previous)]
    segment(approach, abs(previous)*speed/RATE, 0.)
    q = approach[-1]
    trial = [q]
    if not curve(trial, q.yaw, steer, target, speed):
        return dict(reason='insufficient_heading_change_for_ramps')
    end = trial[-1]
    d = dot(g[0]-end.x,g[1]-end.y,v)/math.cos(q.yaw-ax)
    if d<0:
        return dict(reason='would_have_to_start_behind', extra_straight_m=d)
    segment(approach, d, 0.)
    assert curve(approach, q.yaw, steer, target, speed)
    end = approach[-1]
    assert abs(angle(end.yaw-target))<1e-7 and abs(dot(end.x-g[0],end.y-g[1],v))<1e-7
    depth = dot(end.x-g[0],end.y-g[1],n)
    segment(approach, max(0.,final_depth_m-depth), 0.)
    for a,b in zip(approach,approach[1:]):
        assert (b.s-a.s)<=DS+1e-9 and abs(b.steer-a.steer)<=RATE*(b.s-a.s)/speed+1e-8
    return dict(reason='candidate', straight_before_turn_m=q.s+d, extra_straight_m=d,
                final_rear_depth_m=max(final_depth_m, depth), poses=[astuple(p) for p in approach])
