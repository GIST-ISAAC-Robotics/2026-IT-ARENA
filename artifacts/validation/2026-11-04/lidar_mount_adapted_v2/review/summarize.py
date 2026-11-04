"""완료된 실행 영수증/원본을 대조한다. 실행/기준 판정을 바꾸지 않는다."""
from collections import Counter
from pathlib import Path
import json, re, statistics

B=Path(__file__).resolve().parents[1]
def read(p):return json.loads(p.read_bytes()) if p.is_file() and p.stat().st_size else {}
def structure_samples(run,begin,end):
    count=limited=memory=0; caps=[]; modes=Counter(); times=Counter(); last_t=None;last_mode=None
    f=run/'trajectory.jsonl'
    if not f.is_file():return {}
    for line in f.open(encoding='utf8'):
        r=json.loads(line);t=r.get('time');
        if t is None or begin is None or not begin<=t<=end:continue
        part=(r.get('safety') or {}).get('partial_observation') or {}; k=part.get('known_structure') or {}
        if last_t is not None: times[last_mode]+=t-last_t
        mode=part.get('reason','not_evaluated'); last_t,last_mode=t,mode; modes[mode]+=1
        if k.get('evaluated'):
            count+=1; cap=k.get('speed_cap_mps');memory+=bool(k.get('hidden_memory_points'))
            if cap is not None: limited+=1;caps.append(cap)
    return dict(evaluated_samples=count,cap_present_samples=limited,hidden_memory_nonempty_samples=memory,
                minimum_structure_cap_mps=min(caps,default=None),partial_decision_samples=dict(modes),
                partial_decision_time_s={k:round(v,3) for k,v in times.items()})
def main():
    all_runs={}
    for ep in sorted(B.glob('*_eval.json')):
        name=ep.name[:-10];run=B/name; e=read(ep);r=read(run/'report.json'); m=read(B/(name+'_metrics.json'))
        if not r or not (B/(name+'.exit')).is_file():continue
        surface=read(B/(name+'_surface.json'))
        a=dict(validator_pass=r.get('passed'),error=e.get('error'),stop_stable=r.get('stop_stable',(r.get('stop_checks') or {}).get('model_stop_stable')),
               shutdown_clean=r.get('shutdown_clean'),pids=r.get('remaining_test_pids'),source_unchanged=r.get('source_unchanged'),
               sensor_only_inputs=r.get('sensor_only_inputs'),safety_sensor_only_inputs=r.get('safety_sensor_only_inputs'),
               elapsed=(B/(name+'.exit')).read_text().strip(),sensors_sha256=e.get('sensor_profile_sha256'),
               scan_sectors=e.get('blind_sectors_deg'),surface={k:surface.get(k) for k in ('samples','steering_footprint_outside_samples_over_1e-6_m2','wall_wheel_samples','sample_gaps_over_limit')},
               progress_m=e.get('progress_m'),peak_mps=e.get('peak_ground_speed_mps',e.get('peak_speed_mps')),
               zero_command_s=(e.get('decisions') or {}).get('zero_command_s'),zero_by_reason=(e.get('decisions') or {}).get('zero_by_reason'),
               first_move_after_green_s=e.get('first_move_after_green_s'))
        if r.get('case') is None:
            begin=m.get('first_green_sample_s'); lap=e.get('lap_s')
            scopes=structure_samples(run,begin,begin+lap) if begin is not None and lap is not None else {}
            a.update(kind='solo',lap_s=lap,completed_laps=e.get('completed_laps'),best_1s_mps=m.get('best_one_second_mean_mps'),
                     contacts=e.get('collision_samples'),physical_delivery_hz=(r.get('drive_delivery') or {}).get('measured_sim_hz'),
                     structure=scopes)
        else:
            a.update(kind=r['case'],run_valid=e.get('run_valid'),min_gap_m=e.get('min_gap_m'),min_swept_gap_m=e.get('min_swept_gap_m'),
                     contacts=e.get('contacts'),overlaps=e.get('overlap_samples'),actor_scans=e.get('actor_scans'),gates=e.get('gates'))
        all_runs[name]=a
    out=dict(scope='동일 조건 단독·각본별 표본, 실제 차량이나 모든 교통 조건 증명 아님.',runs=all_runs)
    path=B/'comparison_v2.json';path.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    for k,a in all_runs.items():print(k,a['validator_pass'],a.get('lap_s'),a['progress_m'],a.get('min_swept_gap_m'),a['zero_command_s'])
if __name__=='__main__':main()
