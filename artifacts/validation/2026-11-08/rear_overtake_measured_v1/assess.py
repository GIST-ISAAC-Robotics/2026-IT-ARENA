"""실제 두 궤적으로 주행 중 피추월과 그 후 진행을 판정한다. 평가 전용."""
from bisect import bisect_left
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
read = lambda p: json.loads(p.read_bytes())


def interpolate(rows, times, stamp):
    i = bisect_left(times, stamp)
    if i < len(times) and times[i] == stamp:
        return rows[i][1:]
    if not 0 < i < len(times) or times[i]-times[i-1] > .05:
        return None
    a, b = rows[i-1], rows[i]
    u = (stamp-a[0])/(b[0]-a[0])
    dyaw = math.remainder(b[3]-a[3], 2*math.pi)
    return (a[1]+u*(b[1]-a[1]), a[2]+u*(b[2]-a[2]), a[3]+u*dyaw)


def axes(pose, slot):
    x, y, yaw = pose
    angle = slot['yaw_rad']
    dx, dy = x-slot['x'], y-slot['y']
    return (dx*math.cos(angle)+dy*math.sin(angle),
            -dx*math.sin(angle)+dy*math.cos(angle),
            .10*abs(math.cos(yaw-angle))+.075*abs(math.sin(yaw-angle)))


def main(name):
    case = HERE/name
    folder = case/'run1'
    report = read(folder/'report.json')
    raw = read(folder/'trace.json')
    cfg = read(HERE/'new_track_course.json')
    scene = read(ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/scene.json')
    slot = next(s for s in scene['starting_grid']['slots'] if s['index'] == cfg['launch']['grid_slot'])
    green, end = (report['case_window'][k] for k in ('green_sim_s','end_sim_s'))
    with (folder/'actor_poses.jsonl').open() as f:
        ap = [json.loads(line) for line in f if line.strip()]
    at = [a[0] for a in ap]
    assert all(a<b for a,b in zip(at,at[1:]))
    rows = []
    missing = 0
    for row in raw:
        stamp = row['t']
        if not green <= stamp <= end:
            continue
        pose = interpolate(ap, at, stamp)
        if pose is None:
            missing += 1
            continue
        e, l, h = axes([row[k] for k in ('x','y','yaw')], slot)
        a, al, ah = axes(pose, slot)
        delta = a-e
        state = 'rear' if delta < -(ah+h) else ('front' if delta > ah+h else 'alongside')
        rows.append((stamp-green, state, e, al-l, row['ground_speed_mps']))
    rear = next((i for i,r in enumerate(rows) if r[1]=='rear'), None)
    side = next((i for i,r in enumerate(rows) if rear is not None and i>rear and r[1]=='alongside'), None)
    front = next((i for i,r in enumerate(rows) if side is not None and i>side and r[1]=='front'), None)
    alongside = [r for r in rows[(side or 0):front] if r[1]=='alongside'] if side is not None else []
    def one_second(start):
        a, b = (interpolate(ap, at, green+t) for t in (start, start+1))
        return None if a is None or b is None else axes(b,slot)[0]-axes(a,slot)[0]
    windows = [one_second(t) for t in (2.3,3.3,4.3,5.3)]
    conditions = dict(actual_rear_side_front=rear is not None and side is not None and front is not None,
        started_behind=bool(rows and rows[0][1]=='rear'),
        sampling_complete=bool(rows and not missing and rows[0][0]<.15 and end-green-rows[-1][0]<.15
                               and all(0<b[0]-a[0]<.15 for a,b in zip(rows,rows[1:]))),
        on_left=bool(alongside and all(r[3]>0 for r in alongside)),
        ego_moving_during_pass=any(r[2]>=.20 and r[4]>=.20 for r in alongside),
        actor_speed_measured=all(v is not None and abs(v-1.80)<.10 for v in windows))
    longest = span = 0.
    for a,b in zip(rows,rows[1:]):
        span = span+b[0]-a[0] if a[4]<.05 else 0.
        longest = max(longest,span)
    post = rows[-1][2]-rows[front][2] if front is not None else None
    surface = read(case/'surface_run1.json')
    gap = report['evaluation']['gap']
    sw = gap.get('min_swept_lower_bound_m')
    stop = report.get('stop_checks', {})
    wheels = surface['interpretations']
    integration = read(case/'postflight_integrity_run1.json')
    gates = dict(valid=report.get('harness_valid') is True and report.get('run_valid') is True,
        official_rgb=report.get('official_signal_evidence',{}).get('start_after_actual_green') is True,
        stop_clean=bool(stop) and all(stop.values()) and report.get('shutdown_clean') is True,
        source_clean=not integration['changed'] and report.get('source_unchanged') is True,
        process_clean=not (case/'postflight_processes_run1.txt').read_text().strip(),
        contact_free=gap.get('actor_contact_messages_with_ego')==0 and gap.get('sampled_overlap_samples')==0,
        wall_clear=report.get('wall_collision_samples')==0 and surface.get('wall_wheel_samples')==0,
        minimum_gap=gap.get('sampled_finite_complete') is True and gap['min_sampled_gap_m']>=.08,
        swept_gap=sw is not None and sw>=.08 and gap.get('swept_missing_actor_intervals')==0,
        surface=surface.get('samples',0)>0 and surface.get('sample_gaps_over_limit')==0
                and set(wheels)=={'any','center','full'} and all(v['max_off_wheels']<=2 for v in wheels.values()),
        continues_after_pass=post is not None and post>=.50, no_long_stall=longest<=2.5)
    met=dict(observed_speed_windows=windows, initial_s_to_ego=axes(ap[0][1:],slot)[0], alongside_samples=len(alongside),
             alongside_s=[r[0] for r in alongside], first_front_s=rows[front][0] if front is not None else None,
             progress_at_end_m=rows[-1][2] if rows else None, post_pass_progress_m=post,
             peak_mps=max((r[4] for r in rows),default=None),longest_low_speed_s=longest,
             min_gap=gap['min_sampled_gap_m'],swept_min_gap=sw, wheels={k:v['max_off_wheels'] for k,v in wheels.items()})
    result=dict(scene=conditions, established=all(conditions.values()),gates=gates,metrics=met,
                raw_pass=report['passed'],raw_gates=report['gates'],passed=all(conditions.values()) and all(gates.values()))
    (case/'assessment_run1.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    sys.exit(main(sys.argv[1]))
