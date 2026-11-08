"""초기 그리드 간격과 실제 이동 이후 간격을 구별한다. 원 판정은 보존한다."""
import json
from pathlib import Path
import sys

import g1_traffic_eval as ev

HERE=Path(__file__).resolve().parent
read=lambda p:json.loads(p.read_bytes())
rows=lambda p:[json.loads(line) for line in p.read_bytes().splitlines() if line.strip()]


def main(case):
    folder=HERE/case;run=folder/'run1';r=read(run/'report.json');s=read(folder/'surface_run1.json')
    first,end=(r['case_window'][k] for k in ('green_sim_s','end_sim_s'))
    tr=[row for row in read(run/'trace.json') if first<=row['t']<=end]
    moving=next((row['t'] for row in tr if row['ground_speed_mps']>=.10),None)
    raw=read(run/'additional_actors.json')
    actors={**raw['poses'], 'eval_g1b_actor':rows(run/'actor_poses.jsonl')}
    ego=rows(run/'ego_poses.jsonl')
    by_actor={}
    for name,poses in actors.items():
        span=ev.window_tuples(ego,moving or end,end)
        sampled=ev.sampled_gaps(span,poses,.20,.15,ev.POSE_MAX_GAP_S)
        swept=ev.swept_gap_lower_bounds(span,poses,.20,.15,.01,3.,ev.POSE_MAX_GAP_S)
        vals=[v['gap_m'] for v in sampled if v['gap_m'] is not None]
        lows=[v['lower_bound_m'] for v in swept if v['lower_bound_m'] is not None]
        by_actor[name]=dict(samples=len(sampled),intervals=len(swept),minimum_gap_m=min(vals,default=None),
                           minimum_swept_gap_m=min(lows,default=None),complete=bool(vals and lows)
                           and len(vals)==len(sampled) and len(lows)==len(swept))
    span=longest=0.
    for a,b in zip(tr,tr[1:]):
        if moving is None or a['t']<moving:continue
        span=span+b['t']-a['t'] if a['ground_speed_mps']<.05 else 0.
        longest=max(longest,span)
    extra=r['additional_actors']
    gap=r['evaluation']['gap']
    stop=r.get('stop_checks',{})
    integrity=read(folder/'postflight_integrity_run1.json')
    contacts=extra.get('combined_contacts',{})
    # combined_contacts events retain per-pair identity; actor-actor contacts are also failures.
    gates=dict(actual_four_actors=len(actors)==4 and extra.get('actor_count')==3,
        runtime_valid=r.get('run_valid') is True and r.get('harness_valid') is True,
        late_start=r.get('official_signal_evidence',{}).get('start_after_actual_green') is True,
        progress=bool(tr and tr[-1]['progress_m']>=2.),moving=moving is not None,no_long_stall=longest<=2.5,
        no_contacts=gap.get('actor_contact_messages_with_ego')==0 and contacts.get('valid') is True and not contacts.get('events'),
        no_overlap=gap.get('sampled_overlap_samples')==0,
        moving_clearance=all(v['complete'] and v['minimum_gap_m']>=.08 and v['minimum_swept_gap_m']>=.08 for v in by_actor.values()),
        surface=s.get('samples',0)>0 and s.get('sample_gaps_over_limit')==0 and s.get('wall_wheel_samples')==0
            and set(s['interpretations'])=={'any','center','full'} and all(v['max_off_wheels']<=2 for v in s['interpretations'].values()),
        stop_and_shutdown=bool(stop) and all(stop.values()) and r.get('shutdown_clean') is True,
        unchanged=r.get('source_unchanged') is True and not integrity['changed'],
        process_clean=not (folder/'postflight_processes_run1.txt').read_text().strip())
    out=dict(scope='five-car grid representative only; initial spacing and moving clearances are separate; signal occlusion must be reviewed from images',
        gates=gates,passed=all(gates.values()),progress_m=tr[-1]['progress_m'] if tr else None,
        longest_low_speed_after_motion_s=longest,first_motion_after_green_s=moving-first if moving else None,
        by_actor_after_first_motion=by_actor,raw_gates=r['gates'],raw_pass=r['passed'],
        initial_placements=read(HERE/'geometry.json')['placements'])
    (folder/'assessment_run1.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(out,ensure_ascii=False))
    return 0 if out['passed'] else 1


if __name__=='__main__':
    sys.exit(main(sys.argv[1]))
