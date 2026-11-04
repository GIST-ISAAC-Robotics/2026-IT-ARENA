"""원시 통과/실패를 변경하지 않는 단독 주행 사후 판정. 부분 출력도 원본 그대로 보존."""
from bisect import bisect_right
from collections import Counter
from hashlib import sha256
import json
import math
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
RUN=HERE/'run1'
BASE=HERE.parent/'solo_post_free_v1'


def load(f):
    return json.loads(f.read_bytes())


def main():
    report=load(RUN/'report.json')
    metrics=load(HERE/'run1_metrics.json');old=load(BASE/'run1_metrics.json')
    surface=load(HERE/'run1_surface.json'); motor=load(HERE/'run1_motor_audit.json')
    chain=load(HERE/'run1_speed_chain.json'); base_chain=load(HERE/'baseline_speed_chain.json')
    registered=load(HERE/'run1_registered.json')
    altered=[n for n,h in registered['source_sha256'].items() if sha256((ROOT/n).read_bytes()).hexdigest()!=h]
    trace=load(RUN/'trace.json');times=[r['time'] for r in trace]
    green=report.get('start_time_sim_s')
    stops=[r['now_us']*1e-6 for r in load(RUN/'actuation_trace.json') if r.get('reason')=='software_stop'
           and (green is None or r['now_us']*1e-6>=green)]
    end=stops[0] if stops else times[-1]
    reason=Counter();zero=corner_zero=0.;guard_violations=0; footer=None;seq=0; enabled=False;same_steer_notes=0
    for e in map(json.loads,(RUN/'follow_decisions.jsonl').open(encoding='utf8')):
        assert footer is None and e['seq']==seq;seq+=1
        if e['kind']=='header':
            cfg=e['data']['follow_config']
            enabled=bool(cfg.get('solo_straight_budget_trial') is True and not cfg.get('ray_wall_continuity')
                         and cfg.get('solo_route_only') and cfg.get('partial_observation_policy')=='bounded_local_v4')
        elif e['kind']=='footer':
            footer=e['data']
        elif e['kind']=='call':
            i=e['data']['input'];t=i['reference_s'];v,a,d=e['data']['output']
            if green is None or not green<=t<end:
                continue
            why=d.get('solo_straight_budget');reason[why or 'no_trial_detail']+=1
            s=trace[max(0,bisect_right(times,t)-1)]['progress_m']
            if v<=1e-9:
                zero+=i['dt_s']
                if 42<=s<44:corner_zero+=i['dt_s']
            if why=='straight_budget':
                guard_violations+=int(not (v>d['original_speed_mps']>0 and i['speed_mps']>=0 and
                    d['partial_observation']['reason']=='fully_observed' and not d['boundary_from_memory']))
                same_steer_notes+=int(42<=s<44)
    footer_ok=footer is not None and footer['accepted']==footer['written']==seq-1 and not any(
        footer.get(k) for k in ('pending','failed','dropped','capture_errors','worker_alive'))
    wheel=surface['interpretations']['any']['max_off_wheels']
    clean=dict(raw=report.get('passed') is True,complete=report.get('route_complete') is True,
        sensor_only=(report.get('sensor_only_inputs') is True and report.get('safety_sensor_only_inputs') is True),
        actual_signal=report.get('official_signal_evidence',{}).get('passed') is True,
        collision=report.get('collision_samples')==0,road=wheel==0 and surface['sample_gaps_over_limit']==0
            and surface['wall_wheel_samples']==0 and surface['body_outside_samples_over_1e-6_m2']==0,
        motor=motor['passed'] is True,stop=report.get('stop_stable') is True,
        shutdown=report.get('shutdown_clean') is True and report.get('remaining_test_pids')==[],
        input=not altered and report.get('source_unchanged') is True and footer_ok,
        option_used=enabled and reason['straight_budget']>0 and guard_violations==0)
    keys=['lap_s_from_first_green_sample','peak_ground_mps','best_one_second_mean_mps',
          'time_above_95_percent_s','distance_above_95_percent_m','time_ground_at_most_0p05_mps_s']
    body=(chain.get('values') or {}).get('ground')
    clean['ground_to_finish']=bool(body and chain['first_finish_s'] is not None and
                                   abs(chain['end_s']-chain['first_finish_s'])<1e-8)
    summary=dict(checks=clean,driving_checks_passed=all(clean.values()),target_10kmh_reached=(
        body['peak_mps']>=10/3.6 if body else None),
        body_from_first_permission_to_finish_or_stop=dict(prior=base_chain['values']['ground'],current=body),
        profile=report.get('speed_profile'),options_in_actual_header=enabled,
        comparison={k:dict(prior=old.get(k),current=metrics.get(k)) for k in keys},
        speed_budget_reasons=dict(reason),budget_applications_at_42_to_44_m=same_steer_notes,
        zero_raw_target_sample_s=zero,zero_raw_target_42_to_44_m_s=corner_zero,
        raw_exit=(HERE/'runtime.exit').read_text().strip(),changed_sources=altered,
        decoded_raw_video=report.get('video',{}).get('decode_passed'),max_off_wheels=wheel,
        surface_samples=surface['samples'],scope='단독 시뮬 1회. 과거 입력의 조향 동일과 새 궤적의 조향 동일은 다르다. 영상 성공 여부는 주행과 별도.')
    with (HERE/'review.json').open('x',encoding='utf8') as f:
        json.dump(summary,f,indent=2,ensure_ascii=False,allow_nan=False)
    print(json.dumps(summary,ensure_ascii=False))
    return int(not all(clean.values()))


if __name__=='__main__':
    raise SystemExit(main())
