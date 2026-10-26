"""저장된 순차 광선의 옆 actor 교차 진단. 정답은 여기서만 사용한다."""
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import sys

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import thaw


def nearest_before(rows, times, t):
    i = bisect_right(times, t)-1
    if i < 0 or t-times[i] > .05:
        return None
    return rows[i]


def main(mode):
    base = OUT.parent/'e3_onroad_v1'/mode
    paths = [base/n for n in ('ego_poses.jsonl','additional_actors.json','follow_decisions.jsonl')]
    ego = [json.loads(s) for s in paths[0].read_text().splitlines()]
    actor = json.loads(paths[1].read_text())['poses']['eval_e3_side']
    et, at = [v[0] for v in ego], [v[0] for v in actor]
    results, seen = [], set()
    for text in paths[2].open():
        event = json.loads(text)
        if event['kind']=='header':
            cfg = event['data']['motion_config']
        if event['kind']!='call':
            continue
        call = thaw(event['data'])
        inp = call['input']
        scan = inp['scan']
        stamp = scan['sec'] + scan['nanosec']*1e-9
        if stamp in seen or not 3.6 <= stamp <= 6.6:
            continue
        seen.add(stamp)
        hits, missing = [], 0
        for i, distance in enumerate(scan['ranges']):
            angle = scan['angle_min']+i*scan['angle_increment']
            if not (math.isfinite(distance) and scan['range_min'] <= distance <= scan['range_max']):
                continue
            t = stamp+i*scan['time_increment']
            ep, ap = nearest_before(ego, et, t), nearest_before(actor, at, t)
            if ep is None or ap is None:
                missing += 1
                continue
            _, ex, ey, eyaw = ep
            _, ax, ay, ayaw = ap
            x = ex+math.cos(eyaw)*cfg['lidar_x_m']-math.sin(eyaw)*cfg['lidar_y_m']
            y = ey+math.sin(eyaw)*cfg['lidar_x_m']+math.cos(eyaw)*cfg['lidar_y_m']
            x += distance*math.cos(eyaw+cfg['lidar_yaw_rad']+angle)
            y += distance*math.sin(eyaw+cfg['lidar_yaw_rad']+angle)
            lx = math.cos(ayaw)*(x-ax)+math.sin(ayaw)*(y-ay)
            ly = -math.sin(ayaw)*(x-ax)+math.cos(ayaw)*(y-ay)
            # 판정용 식별이 아니라 정답 사각형 가까이에 떨어진 점의 사후 분류다.
            if abs(lx)<=.115 and abs(ly)<=.09:
                hits.append(math.degrees(angle))
        detail = call['output'][2]
        results.append({'scan_first_s':stamp,'first_decision_s':inp['reference_s'],
            'actor_near_endpoint_count':len(hits),'missing_pose_joins':missing,
            'angle_range_deg':[min(hits),max(hits)] if hits else None,
            'decision_speed_mps':call['output'][0], 'reason':detail.get('reason'),
            'swept_clearance_m':detail.get('swept_clearance_m')})
    report={'mode':mode, 'scope':'offline raw ray endpoints within actor box +15mm tolerance at acquisition time; not online identity/ground truth input',
        'pose_join':'latest before, max50ms; actual pose stream steps about2/10ms and set_pose jumps remain',
        'results':results,'sources':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}
    with (OUT/f'{mode}_hits.json').open('x') as stream:
        json.dump(report,stream,indent=2)
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    if len(sys.argv)!=2 or sys.argv[1] not in ('g1','r0plus'):
        raise SystemExit('expected g1 or r0plus')
    main(sys.argv[1])
