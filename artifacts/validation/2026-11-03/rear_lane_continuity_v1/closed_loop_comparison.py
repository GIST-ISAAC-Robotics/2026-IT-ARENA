"""같은 각본 2회의 실제 궤적 비교. 평가용 자세가 제어기에 들어가지 않는다."""
import hashlib
import json
import math
from zipfile import ZipFile
import yaml
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent


def read_scene(name):
    d = HERE.parent/name
    run = d/'run1'
    report = json.loads((run/'report.json').read_bytes()); ev = report['evaluation']
    goal = json.loads((d/'run1_goal.json').read_bytes())
    phase = goal['phase']; green = ev['t_green']; end = report['case_window']['end_sim_s']
    trace = [r for r in json.loads((run/'trace.json').read_bytes()) if green <= r['t'] <= end]
    poses = np.asarray([json.loads(l)[:4] for l in (run/'actor_poses.jsonl').open(encoding='utf-8')])
    if len(poses) < 2 or np.any(np.diff(poses[:,0]) < 0): raise ValueError('invalid actor pose time')
    marks=[]
    for t in (0.5,1.7,2.5,3.2,4.4,4.56,5.2,5.8):
        r = min(trace,key=lambda x:abs(x['t']-green-t))
        stamp=r['t']
        if not poses[0,0] <= stamp <= poses[-1,0]: raise ValueError('pose coverage')
        ax,ay=(float(np.interp(stamp,poses[:,0],poses[:,i])) for i in (1,2))
        a=float(np.interp(stamp,poses[:,0],np.unwrap(poses[:,3])))
        dx,dy=r['x']-ax,r['y']-ay
        marks.append(dict(t=stamp-green, progress_m=r['progress_m'], body_mps=r['ground_speed_mps'],
            drive_mps=r['drive_speed_mps'], safe_mps=r['safe_speed_mps'], sampled_gap_m=r['sampled_gap_m'],
            ego_left_of_actor_m=-math.sin(a)*dx+math.cos(a)*dy,
            ego_ahead_of_actor_m=math.cos(a)*dx+math.sin(a)*dy))
    applied=[]; zero_s=0.; first_response=None
    returned = phase['returned'][0]['stamp'] if phase['established'] else None
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as f:
        header=json.loads(next(f))['data']
        for line in f:
            e=json.loads(line)
            if e['kind']!='call': continue
            data=e['data']; inp=data['input']; t=inp['reference_s']
            if not green <= t <= end: continue
            speed,steer,detail=data['output']; normal=detail.get('normal') or detail
            pref=normal.get('current_lane_preference') or {}
            if pref.get('applied'):
                applied.append(dict(t=t-green,seq=e['seq'],dt=inp['dt_s'],speed=speed,steering=steer,
                                    replaced=pref.get('previous_output'),faster=pref.get('faster')))
            if speed <= 1e-9: zero_s+=inp['dt_s']
            sc=inp['scan']; stamp=sc['sec']+sc['nanosec']*1e-9
            right=(normal.get('selector_evidence') or {}).get('right') or {}
            if returned is not None and stamp>=returned-1e-7 and first_response is None and right.get('present'):
                first_response=dict(t=t-green,scan_t=stamp-green,mode=normal.get('selector_mode'),
                                    command=[speed,steer],reason=normal.get('selector_reason'))
    contact=ev['gap']['first_contact_sim_s']
    with ZipFile(run/'source_snapshot.zip') as z:
        vehicle=yaml.safe_load(z.read('src/arena_description/config/vehicle.yaml'))['vehicle']
    enc=vehicle['sensors']['drive_feedback']['drive_motor_encoder']; drive=vehicle['drivetrain']
    quantum=math.tau*drive['wheel_radius_m']*enc['sample_rate_hz']/(enc['counts_per_motor_revolution']*drive['motor']['gear_ratio'])
    headroom=header['follow_config'].get('current_lane_headroom_mps')
    if header['follow_config'].get('current_lane_preference_trial') and not math.isclose(headroom,quantum,rel_tol=1e-12):
        raise ValueError('recorded lane headroom disagrees with captured encoder configuration')
    return dict(scenario_sha256=hashlib.sha256((d/'scenario.json').read_bytes()).hexdigest(),
        passed=goal['passed'],failed=[k for k,v in goal['checks'].items() if not v],
        phase_established=phase['established'], first_reapproach_response=first_response,
        first_contact_after_green_s=contact-green if contact is not None else None,
        metrics=goal['metrics'], contacts=ev['gap']['actor_contact_messages_with_ego'],
        overlaps=ev['gap']['sampled_overlap_samples'], zero_command_sample_s=zero_s,
        lane_headroom_mps=headroom, encoder_sample_hz=enc['sample_rate_hz'], configured_quantum_mps=quantum,
        preference_calls=len(applied), preference_sample_s=sum(x['dt'] for x in applied),
        first_preference=applied[:1],marks=marks)


if __name__=='__main__':
    result={k:read_scene('right_reapproach_'+k) for k in ('v2','v3')}
    if result['v2']['scenario_sha256']!=result['v3']['scenario_sha256']:
        raise ValueError('different scenario bytes')
    (HERE/'closed_loop_comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:{a:b for a,b in v.items() if a not in ('marks','first_preference')} for k,v in result.items()}))
