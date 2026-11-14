"""actor0 평가. 기존 수치/coverage/표면/STOP 계약을 재사용한다. 제어에 연결하지 않는다."""
import math
import xml.etree.ElementTree as ET
from shapely.strtree import STRtree
import g1_traffic_eval as ev
from g1_course_eval import wheel_center_surface
from solo_finalize import finalize_verdict

FACILITIES={'eval_r406_trial_old610_signal_support','eval_r406_trial_old610_marker20_support',
            'eval_r406_trial_old610_marker30_support'}

def no_actor_evidence(runtime_world):
    world=ET.fromstring(runtime_world).find('world')
    names={m.get('name') for m in world.findall('model')} if world is not None else set()
    expected={'it_arena_track_static',*FACILITIES}
    return dict(passed=expected.issubset(names) and names<=expected|{'arena_car'},
                names=sorted(names),actor_count=0 if names<=expected|{'arena_car'} else None,
                facility_count=len(names&FACILITIES))

def evaluate_run(*,config,case,mode,t_green,trace,ego,actor,contacts,commands,stop_start,
                 schedule_pose,centerline,s0,road,runtime_world,contact_evidence,obstacles):
    assert not actor and not commands and not contacts and schedule_pose is None
    result=dict(t_green=t_green,evaluator_version='r457_solo_from_r452',
                no_actor=no_actor_evidence(runtime_world),
                actor_checks_applicability='not_applicable_actor_count_zero',
                contact_evidence=contact_evidence)
    if t_green is None or not ev._finite(t_green) or not ev._finite(stop_start):
        return {**result,'gates':{'green_and_stop_observed':False},'harness':{'valid':False}}
    t_end=stop_start
    checks=dict(ego_finite_ordered=ev._pose_rows_valid(ego),
        trace_finite_ordered=bool(trace) and all(
            all(ev._finite(r.get(k)) for k in ('t','ground_speed_mps','safe_speed_mps','x','y','yaw','drive_speed_mps'))
            and isinstance(r.get('autonomy'),dict) and r['ground_speed_mps']>=0 for r in trace)
            and all(b['t']>a['t'] for a,b in zip(trace,trace[1:])))
    result['data_checks']=checks
    if not all(checks.values()):
        return {**result,'harness':{'checks':checks,'valid':False},'gates':{'data_finite_ordered':False}}
    last_time=ego[-1][0]
    active=ev.window_tuples(ego,t_green,t_end)
    all_after=ev.window_tuples(ego,t_green,last_time)
    rows=ev.window(trace,t_green,t_end)
    coverage=dict(
        ego=ev.stream_coverage([r[0] for r in all_after],t_green,t_end,ev.POSE_MAX_GAP_S),
        trace=ev.stream_coverage([r['t'] for r in rows],t_green,t_end,ev.TRACE_MAX_GAP_S))
    geometry=ev.ego_geometry(config)
    progress,jumps=ev.progress_series(centerline,active)
    distance=progress[-1]['progress_m'] if progress else None
    wheels=wheel_center_surface(road,all_after,geometry['wheelbase_m'],geometry['track_width_m'])
    tree=STRtree(obstacles)
    footprints=[ev.rectangle(x,y,yaw,geometry['length_m'],geometry['width_m']) for _,x,y,yaw in all_after]
    outside=sum(p.difference(road).area>1e-6 for p in footprints) if road is not None else None
    wall=sum(any(p.intersection(obstacles[int(i)]).area>1e-6 for i in tree.query(p)) for p in footprints)
    moved=[r['t'] for r in rows if r['ground_speed_mps']>.10]
    stall=ev.longest_true_interval(ev.window(rows,moved[0],t_end),lambda r:r['ground_speed_mps']<.05,
          max_gap_s=ev.TRACE_MAX_GAP_S) if moved else None
    stationary_total=sum(b['t']-a['t'] for a,b in zip(rows,rows[1:])
        if b['t']-a['t']<=ev.TRACE_MAX_GAP_S and a['ground_speed_mps']<.05 and b['ground_speed_mps']<.05)
    full=distance is not None and distance>=centerline.lap and jumps==0
    harness={**checks,'ego_window_covered':coverage['ego']['valid'],'trace_window_covered':coverage['trace']['valid'],
             'stop_request_within_registered_bound':t_green<=t_end<=t_green+case['duration_s']+ev.TRACE_MAX_GAP_S,
             'no_actor_with_facilities':result['no_actor']['passed']}
    result.update(coverage=coverage,harness=dict(checks=harness,valid=all(harness.values())),ego_geometry=geometry,
        progress=dict(at_window_end_m=distance,projection_jumps=jumps),
        full_course_progress=dict(valid=bool(progress) and jumps==0,progress_m=distance,lap_length_m=centerline.lap),
        wheel_centers=wheels,road=dict(outside_samples=outside,scope='configured conservative footprint vs SDF road; historical body-only gate reported separately'),
        dense_wall_overlap=dict(samples=len(footprints),overlap_samples=wall,scope='sampled planar configured footprint; not continuous physical contact proof'),
        historical_road_only_gate=outside is not None and outside<=config['acceptance']['outside_road_samples'],
        case_metrics=dict(first_motion_sim_s=moved[0] if moved else None,longest_stationary_s=stall,
                          stationary_total_sim_s=stationary_total,elapsed_since_green_s=t_end-t_green,
                          mean_progress_speed_mps=distance/(t_end-t_green) if distance is not None and t_end>t_green else None),
        controller_mode=ev.controller_mode_summary([r['autonomy'] for r in rows if r['autonomy'].get('behavior') is not None]),
        gates=dict(green_observed=True,data_coverage=all(c['valid'] for c in coverage.values()),full_lap=full,
                   two_wheel_surface=wheels['valid'] and wheels['three_plus_samples']==0,
                   no_permanent_stall=stall is not None and stall<=config['acceptance'][case['name']]['maximum_stationary_while_lead_moves_s']))
    return result
