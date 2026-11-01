"""원래 추종 판정과 별도로 155초의 경기 진행·접촉·정체를 보존한다. 제어 입력 아님."""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
import numpy as np

BASE=Path(__file__).resolve().parent
PRIOR=BASE.parents[1]/'2026-10-31/g1_continuous_prefix_course_v1'
sys.path.insert(0,str(PRIOR))
from progress_eval import assess as nonprogress
from restart_evidence import observed_restart


def measure(run):
    r=json.loads((run/'report.json').read_text(encoding='utf-8'))
    g=r['case_window']['green_sim_s'];end=r['case_window']['end_sim_s']
    if g is None or end is None:raise ValueError('실제 녹색/각본 창 없음')
    ev=r['evaluation'];evg=ev['gates']
    with (run/'trajectory.jsonl').open(encoding='utf-8') as f:
        trace=[x for x in map(json.loads,f) if g<=x['t']<=end]
    t=np.array([a['t'] for a in trace]);v=np.array([a['ground_speed_mps'] for a in trace])
    if len(t)<2 or not np.all(np.diff(t)>0) or not np.isfinite(v).all():raise ValueError('차체 궤적 누락')
    centers=[(x['x'],x['y']) for x in trace]
    transitions=[];digest=hashlib.sha256()
    with (run/'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            digest.update(line);d=json.loads(line)
            if d['kind']!='call':continue
            a=d['data'];s=a['input']['reference_s'];detail=a['output'][2]
            if not g<=s<=end:continue
            state=detail.get('handoff_state')
            if not transitions or transitions[-1]['state']!=state:
                transitions.append(dict(time_s=s,state=state,reason=detail.get('reason')))
    if digest.hexdigest()!=r['follow_decisions']['sha256']:raise ValueError('판단 원시 파일 변경')
    with (run/'actor_poses.jsonl').open(encoding='utf-8') as f:restart=observed_restart(list(map(json.loads,f)),transitions,g)
    pa=nonprogress(run)
    additional=r['additional_actors']
    return dict(source_unchanged=r['source_unchanged'],run_valid=r['run_valid'],raw_gate_pass=r['passed'],
        raw_failed=[k for k,val in r['gates'].items() if val is not True],
        actual_signal=r.get('team_signal_evidence'),control_profile=r.get('handoff_reserve'),
        progress_m=ev['full_course_progress']['progress_m'],full_lap=evg['full_lap'],
        controller_states=transitions,opponent_restart=restart,explicit_stop=r['stop_checks'],
        contact=ev['gap']['actor_contact_messages_with_ego'],min_gap_m=ev['gap']['min_sampled_gap_m'],
        min_sweep_m=ev['gap']['min_swept_lower_bound_m'],any_actor_contacts=additional['combined_contacts'],
        additional={k:dict(min_gap_m=a['min_sampled_gap_m'],min_sweep_m=a['min_swept_lower_bound_m'],
                    gates=a['gates']) for k,a in additional['actors'].items()},
        actual_body=dict(max_mps=float(max(v)),sample_interval_max_s=float(max(np.diff(t))),
            sample_path_length_m=sum(math.dist(a,b) for a,b in zip(centers,centers[1:])),
            mean_speed_over_sampled_time_mps=float(np.sum((v[:-1]+v[1:])*.5*np.diff(t))/(t[-1]-t[0])),
            first_sample_after_green_s=t[0]-g,last_sample_before_end_s=end-t[-1]),
        nonprogress_10_31_definition=pa,clean_shutdown=r['shutdown_clean'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path)
    p.add_argument('out',type=Path);a=p.parse_args()
    result=measure(a.run)
    with a.out.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
