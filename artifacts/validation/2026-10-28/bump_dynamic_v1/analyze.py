"""정답 자세는 통과 평가에만 사용한다. 제어 출력/판정을 고치지 않는다."""
import argparse
import json
import math
from pathlib import Path


def rotation(q):
    x, y, z, w = (float(q.get(k, 0)) for k in ('x','y','z','w'))
    if not all(math.isfinite(v) for v in (x,y,z,w)) or abs(x*x+y*y+z*z+w*w-1) > .01:
        raise ValueError('invalid pose quaternion')
    return ((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
            (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
            (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))


def pose_metrics(poses, bump):
    c, s = math.cos(bump['yaw_rad']), math.sin(bump['yaw_rad'])
    wheels = {name: [] for name in ('front_left','front_right','rear_left','rear_right')}
    attitude = []
    for pose in poses:
        p = pose['position']; r = rotation(pose['orientation'])
        pitch = math.asin(max(-1., min(1., -r[2][0])))
        roll = math.atan2(r[2][1], r[2][2])
        dx, dy = p['x']-bump['x'], p['y']-bump['y']
        attitude.append(dict(time=pose['time'], along=c*dx+s*dy, lateral=-s*dx+c*dy,
                             z=p.get('z',0.), pitch_deg=math.degrees(pitch), roll_deg=math.degrees(roll)))
        for name, records in wheels.items():
            offset = (.0675 if name.startswith('front') else -.0675,
                      .0675 if name.endswith('left') else -.0675, .0325)
            wx = p['x'] + sum(r[0][i]*offset[i] for i in range(3)) - bump['x']
            wy = p['y'] + sum(r[1][i]*offset[i] for i in range(3)) - bump['y']
            records.append((pose['time'], c*wx+s*wy, -s*wx+c*wy))
    evidence = {}
    for name, rows in wheels.items():
        near = [v for v in rows if abs(v[1]) <= .025+.0325]
        before = [v for v in rows if v[1] < -.025-.0325]
        after = [v for v in rows if v[1] > .025+.0325]
        evidence[name] = dict(before=bool(before), after=bool(after), crossing_samples=len(near),
            max_abs_lateral_m=max((abs(v[2]) for v in near), default=None),
            first_crossing_s=near[0][0] if near else None, last_crossing_s=near[-1][0] if near else None,
            passed=bool(before and after and near and before[0][0] < near[0][0] < after[-1][0]
                        and max(abs(v[2]) for v in near)+.006 <= .225))
    near_body = [r for r in attitude if abs(r['along']) < .3]
    return dict(wheels=evidence, all_wheels_passed=all(v['passed'] for v in evidence.values()),
        crossing_pose_samples=len(near_body),
        max_abs_pitch_deg=max((abs(r['pitch_deg']) for r in near_body), default=None),
        max_abs_roll_deg=max((abs(r['roll_deg']) for r in near_body), default=None),
        near_attitude=near_body)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('output',type=Path); args=parser.parse_args()
    out=args.output
    if (out/'bump_analysis.json').exists(): raise ValueError('preserve previous analysis')
    fixture=Path(__file__).resolve().parents[2]/'2026-10-27/official_departure_v2/fixture'
    scene=json.loads((fixture/'scene.json').read_text(encoding='utf-8'))
    report=json.loads((out/'report.json').read_text(encoding='utf-8'))
    poses=json.loads((out/'bump_pose_trace.json').read_text(encoding='utf-8'))
    result=pose_metrics(poses,scene['speed_bumps']['bumps'][0])
    result.update(base_passed=report.get('passed'), limited_passed=bool(report.get('passed') and result['all_wheels_passed']),
        scope='pose-based geometry crossing and original evaluator; not actual contact-force or hardware proof')
    for kind in ('imu','scan'):
        rows=json.loads((out/f'bump_{kind}_trace.json').read_text(encoding='utf-8'))
        times=[r['time'] for r in rows]
        result[kind+'_delivery']=dict(samples=len(times), nonincreasing=sum(b<=a for a,b in zip(times,times[1:])),
            max_gap_s=max((b-a for a,b in zip(times,times[1:])), default=None))
    (out/'bump_analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='near_attitude'},ensure_ascii=False))
    return 0


if __name__=='__main__': raise SystemExit(main())
