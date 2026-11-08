"""경기거리 대표 사전 목표와 원 뒤추종 목표를 구별한다."""
from pathlib import Path
import json
import sys

HERE=Path(__file__).resolve().parent
read=lambda p:json.loads(p.read_bytes())


def main(name):
    f=HERE/name;run=f/'run1';r=read(run/'report.json');s=read(f/'surface_run1.json')
    e=r['evaluation'];extra=r.get('additional_actors',{});course=e.get('full_course_progress',{})
    gap=e['gap'];stop=r.get('stop_checks',{})
    t0,end=(r['case_window'][k] for k in ('green_sim_s','end_sim_s'))
    tr=[row for row in read(run/'trace.json') if t0<=row['t']<=end]
    start=next((row['t'] for row in tr if row['ground_speed_mps']>.10),None)
    duration=longest=0.
    for a,b in zip(tr,tr[1:]):
        if start is None or a['t']<start:continue
        duration=duration+b['t']-a['t'] if a['ground_speed_mps']<.05 else 0.
        longest=max(longest,duration)
    result=dict(valid=r.get('run_valid') is True and r.get('harness_valid') is True,
        five_laps=course.get('valid') is True and course.get('progress_m',0)>=5*44.64,
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
    lap_times=[]
    for n in range(1,6):
        cross=next((a['t']-t0+(b['t']-a['t'])*(n*44.64-a['progress_m'])/(b['progress_m']-a['progress_m'])
                    for a,b in zip(tr,tr[1:]) if a['progress_m']<n*44.64<=b['progress_m']),None)
        lap_times.append(cross)
    out=dict(lap_crossings_from_green_s=lap_times, distance_timing_scope='grid3 continuous distance, not finish-line timing',gates=result,passed=all(result.values()),course=course,longest_low_speed_s=longest,
        peak_mps=max((row['ground_speed_mps'] for row in tr),default=None),
        min_gap=gap['min_sampled_gap_m'],min_swept_gap=gap['min_swept_lower_bound_m'],
        raw_pass=r['passed'],raw_gates=r['gates'])
    (f/'assessment_run1.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(out,ensure_ascii=False));return 0 if out['passed'] else 1


if __name__=='__main__':sys.exit(main(sys.argv[1]))
