"""코너 내부 정지 일정/경로·표본 평가의 사전 확인. 제어 입력 아님."""
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
import validate_g1_traffic as h
import g1_traffic_eval as ev


def hold_geometry(rows, poses, first, last, initial_yaw, rules):
    valid = [r for r in rows if first <= r['t'] <= last]
    times = [p[0] for p in poses]
    yaws = np.unwrap([p[3] for p in poses])
    annotated = []
    for r in valid:
        index = ev._latest_index(times,r['t'])
        covered = index >= 0 and r['t']-times[index] <= ev.POSE_MAX_GAP_S+1e-9
        turn = abs(math.degrees(yaws[index]-initial_yaw)) if covered else None
        in_curve = covered and rules['minimum_heading_change_during_hold_deg'] <= turn <= rules['maximum_heading_change_during_hold_deg']
        held = r['ground_speed_mps'] < .01 and r['safe_speed_mps'] == 0 and r['autonomy'].get('behavior') == 'HOLD'
        annotated.append({'t':r['t'],'held_in_curve':bool(in_curve and held),'turn_deg':turn,'pose_covered':covered})
    duration=ev.longest_true_interval(annotated,lambda r:r['held_in_curve'],max_gap_s=ev.TRACE_MAX_GAP_S)
    return {'longest_hold_in_curve_s':duration, 'passed':duration>=rules['minimum_continuous_hold_in_curve_s'],
        'all_hold_trace_poses_covered':bool(annotated) and all(r['pose_covered'] for r in annotated),
        'samples':len(annotated), 'matching_samples':sum(r['held_in_curve'] for r in annotated)}


def preflight():
    path=Path('config/tests/g1_curve_stop_restart_v1.json')
    cfg=h.load_config(path)
    case=h.case_named(cfg,'curve_stop_restart')
    line,scene=h.load_centerline()
    start=h.actor_start_s(cfg,scene,0,case)
    road=h.road_polygon(h.DEMO_WORLD/'world.sdf')
    walls=h.obstacle_polygons(h.DEMO_WORLD/'world.sdf')
    checked=h.check_route(cfg,line,case,start,road,walls)
    stop_s=start+ev.actor_travel_m(case['schedule'],22.)
    initial=line.pose(start)[2]
    yaw=np.unwrap([line.pose(s)[2] for s in np.linspace(start,stop_s,100)])
    turn=abs(math.degrees(yaw[-1]-initial))
    assert 30. < turn < 150.,turn
    assert ev.actor_travel_m(case['schedule'],40.) == 12.25
    assert ev.actor_speed_mps(case['schedule'],22.5)==ev.actor_speed_mps(case['schedule'],26.5)==0.
    assert ev.actor_speed_mps(case['schedule'],27.1)==.35
    rules=cfg['acceptance']['curve_stop_geometry']
    times=np.arange(0,2.01,.01)
    rows=[{'t':float(t),'ground_speed_mps':0.,'safe_speed_mps':0.,'autonomy':{'behavior':'HOLD'}} for t in times]
    cases={}
    for angle,expect in [(0.,False),(math.pi/2,True),(math.pi,False)]:
        poses=[(float(t),0.,0.,angle) for t in times]
        result=hold_geometry(rows,poses,0.,2.,0.,rules)
        assert result['passed']==expect
        cases[str(angle)]=result
    gaps=hold_geometry(rows,[(0.,0.,0.,math.pi/2),(2.,0.,0.,math.pi/2)],0.,2.,0.,rules)
    assert not gaps['passed'] and not gaps['all_hold_trace_poses_covered']
    return {'config_sha256':hashlib.sha256(path.read_bytes()).hexdigest(), 'route':checked,
        'actor_stop_s_m':stop_s,'actor_stop_turn_deg':turn,'hold_geometry_checks':cases,'pose_gap_check':gaps}


if __name__=='__main__':
    out=Path(sys.argv[1])
    assert not out.exists()
    result=preflight()
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(result,ensure_ascii=False))
