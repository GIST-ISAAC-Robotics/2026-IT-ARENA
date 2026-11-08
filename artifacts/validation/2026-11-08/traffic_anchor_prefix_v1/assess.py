"""44초 코너 구간 사전 목표. 원 raw_gates와 보충 연속 진행 평가를 구별한다."""
from pathlib import Path
import json
import sys
import validate_g1_traffic as vt
from g1_course_eval import full_course_progress
from g1_traffic_eval import window_tuples

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
WORLD=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
MINIMUM_PROGRESS_M=21.5
read=lambda p:json.loads(p.read_bytes())
lines=lambda p:[json.loads(x) for x in p.read_text(encoding='utf-8').splitlines() if x.strip()]


def supplementary_progress(run, r, t0, end):
    """조밀 자세의 연속 호길이. first_curve 원평가의 반 랩 unwrap 한계를 보충한다(평가 전용)."""
    config=vt.load_config(HERE/'new_track_course.json');case=config['cases'][0]
    line,scene=vt.load_centerline(WORLD)
    ref=vt.actor_start_s(config,scene,0,case)
    ego,actor=lines(run/'ego_poses.jsonl'),lines(run/'actor_poses.jsonl')
    course=full_course_progress(line,window_tuples(ego,t0,end),window_tuples(actor,t0-.11,end),ref,.20)
    return dict(course,reference_s_m=ref,lap_length_m=line.lap,ego_samples=len(ego),actor_samples=len(actor),
        source='scripts/g1_course_eval.full_course_progress on run1/ego_poses.jsonl, actor_poses.jsonl (primary actor)',
        scope='supplementary evaluation; separate from raw report gates')


def main(name):
    f=HERE/name;run=f/'run1';r=read(run/'report.json');s=read(f/'surface_run1.json')
    e=r['evaluation'];extra=r.get('additional_actors',{})
    gap=e['gap'];stop=r.get('stop_checks',{})
    t0,end=(r['case_window'][k] for k in ('green_sim_s','end_sim_s'))
    course=supplementary_progress(run,r,t0,end)
    tr=[row for row in read(run/'trace.json') if t0<=row['t']<=end]
    start=next((row['t'] for row in tr if row['ground_speed_mps']>.10),None)
    duration=longest=0.;longest_from=None;run_from=None
    for a,b in zip(tr,tr[1:]):
        if start is None or a['t']<start:continue
        if a['ground_speed_mps']<.05:
            run_from=a['t'] if duration==0. else run_from
            duration+=b['t']-a['t']
        else:
            duration=0.
        if duration>longest:longest,longest_from=duration,run_from
    result=dict(valid=r.get('run_valid') is True and r.get('harness_valid') is True,
        prefix_progress=course.get('valid') is True and (course.get('progress_m') or 0)>=MINIMUM_PROGRESS_M,
        no_contacts=gap.get('actor_contact_messages_with_ego')==0 and extra.get('combined_contacts',{}).get('valid') is True and not extra.get('combined_contacts',{}).get('events'),
        minimum_gap=gap.get('sampled_finite_complete') is True and gap['min_sampled_gap_m']>=.08,
        swept_gap=gap.get('swept_missing_actor_intervals')==0 and gap.get('min_swept_lower_bound_m') is not None and gap['min_swept_lower_bound_m']>=.08,
        additional_valid=extra.get('harness_valid') is True and extra.get('behavior_gates_passed') is True and extra.get('actor_count')==1,
        no_overlap=gap.get('sampled_overlap_samples')==0,
        surface=s.get('samples',0)>0 and s.get('sample_gaps_over_limit')==0 and s.get('wall_wheel_samples')==0
            and set(s['interpretations'])=={'any','center','full'} and all(v['max_off_wheels']<=2 for v in s['interpretations'].values()),
        stop_and_shutdown=bool(stop) and all(stop.values()) and r.get('shutdown_clean') is True,
        source_unchanged=r.get('source_unchanged') is True and not read(f/'postflight_integrity_run1.json')['changed'],
        process_clean=not (f/'postflight_processes_run1.txt').read_text().strip(),
        sustained_motion=start is not None and longest<=2.5)
    out=dict(gates=result,passed=all(result.values()),minimum_prefix_progress_m=MINIMUM_PROGRESS_M,
        supplementary_course=course,longest_low_speed_s=longest,longest_low_speed_from_sim_s=longest_from,
        first_motion_sim_s=start,
        peak_mps=max((row['ground_speed_mps'] for row in tr),default=None),
        min_gap=gap['min_sampled_gap_m'],min_swept_gap=gap['min_swept_lower_bound_m'],
        raw_case_metrics=e.get('case_metrics'),raw_pass=r['passed'],raw_gates=r['gates'],
        scope='44 s registered corner prefix only; not a full lap, race or real-vehicle result')
    (f/'assessment_run1.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(out,ensure_ascii=False));return 0 if out['passed'] else 1


if __name__=='__main__':sys.exit(main(sys.argv[1]))
