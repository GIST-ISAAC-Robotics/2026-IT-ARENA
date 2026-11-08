"""관측 간극으로만 만드는 국소 운동학 비교. 지도·정답 위치·조향 발행 없음."""
from dataclasses import dataclass, astuple
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
OBS = HERE.parent / 'p22_sensor_preview_v1'
ROOT = HERE.parents[3]
WB, RATE, DS = .135, 3., .01
angle = lambda a: math.atan2(math.sin(a), math.cos(a))
dot = lambda x,y,v: x*v[0]+y*v[1]
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

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

def turn(hit, previous, radius, speed):
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
    segment(approach, max(0.,.25-depth), 0.)
    for a,b in zip(approach,approach[1:]):
        assert (b.s-a.s)<=DS+1e-9 and abs(b.steer-a.steer)<=RATE*(b.s-a.s)/speed+1e-8
    return dict(reason='candidate', straight_before_turn_m=q.s+d, extra_straight_m=d,
                final_rear_depth_m=max(.25, depth), poses=[astuple(p) for p in approach])

def epochs(rows):
    last = None
    count = 0
    for r in rows:
        good = len(r['right']['notches'])==1
        adjacent = last is not None and 0 < r['t']-last<=.16
        count = (count+1 if adjacent else 1) if good else 0
        if count == 2:
            yield r
        last = r['t']

def main():
    result={}
    for name in ('r11','r27'):
        op = OBS / f'observations_{name}.json'
        obs = json.loads(op.read_bytes())
        rows = {r['t']:r for r in epochs(obs['rows'])}
        source = ROOT / obs['source']
        assert digest(source)==obs['sensor_sha256']
        paths=[]
        with source.open(encoding='utf8') as stream:
            for line in stream:
                e=json.loads(line)
                if e['kind']!='call':continue
                d=e['data']['input']
                r=rows.pop(d['reference_s'],None)
                if r is None:continue
                h=r['right']['notches'][0]
                for radius in (.36,.45):
                    for policy, speed in [('low',.65),('measured',d['speed_mps'])]:
                        out=turn(h,d['previous_steering_rad'],radius,speed) if speed>0 else dict(reason='nonpositive_speed')
                        paths.append(dict(t=r['t'],radius_m=radius,speed_mode=policy,
                            path_speed_mps=speed,measured_speed_mps=d['speed_mps'],
                            distance_to_slow_m=max(0.,d['speed_mps']**2-.65**2)/4,**out))
        assert not rows and digest(source)==obs['sensor_sha256']
        result[name]=dict(source=obs['source'],sensor_sha256=obs['sensor_sha256'],observation_sha256=digest(op),paths=paths)
    (HERE/'paths.json').write_text(json.dumps(result,separators=(',',':'))+'\n',encoding='utf8')

if __name__=='__main__':main()
