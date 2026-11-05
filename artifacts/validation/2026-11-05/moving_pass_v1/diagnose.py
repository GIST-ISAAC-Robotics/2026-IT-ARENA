"""첫 추월 실패의 재개 지점. 센서 결정과 실제 자세는 종료 뒤 시각으로만 연결한다."""
from collections import Counter
import json
from pathlib import Path
import sys
from review import lines,read

def main(folder):
    here=Path(__file__).resolve().parent/folder;run=here/'run1'; report=read(run/'report.json')
    tg,te=report['case_window']['green_sim_s'],report['case_window']['end_sim_s']
    episodes=[];reasons=Counter();per_source={};last=None
    band_counts=Counter();band_examples={};holds=Counter()
    for line in lines(run/'follow_decisions.jsonl'):
        if line['kind']!='call':continue
        d=line['data'];t=d['input']['reference_s']; speed,steer,detail=d['output'];m=detail.get('moving_pass',{});target=m.get('target',{});entry=m.get('entry',{})
        if not tg<=t<=te:continue
        key=(detail.get('behavior'),detail.get('reason'),m.get('reason'),m.get('phase'),
             target.get('reason'),target.get('rear_clear'),entry.get('reason'))
        if key!=last:
            episodes.append(dict(start_s=round(t-tg,3),key=key,count=0,end_s=None,first_speed=speed,
                first_steering=steer,first_reference=entry.get('reference'),
                first_candidates=[{k:c.get(k) for k in ('reason','offset_m','target_speed_mps','motion_region','steering_rad','command_clearance_m')} for c in entry.get('candidates',[])][:3]))
            last=key
        episodes[-1].update(end_s=round(t-tg,3),count=episodes[-1]['count']+1)
        if detail.get('reason')=='moving_pass_current_path_unavailable':
            holds[entry.get('reason','not_reported')]+=1
            reasons.update(c.get('reason') for c in entry.get('candidates',[]))
        band=entry.get('rejoin_band')
        if band:
            why=('no_current_rear' if not band.get('moving_target_rear_observed') else
                 'occupied_band' if band.get('current_occupied_returns') else
                 'unsupported_front' if band.get('front_unobserved') else 'supported' if band.get('supported') else 'other')
            band_counts[why]+=1
            if why not in band_examples:band_examples[why]=dict(t=round(t-tg,3),data=band)
        # 동일 광선 재사용은 복귀 확인으로 세지 않고, 마지막 상태가 아닌 첫 관측만 보관한다.
        if target.get('fresh'):
            per_source[round(t,3)]=dict(t=round(t-tg,3),bounds=target.get('observed_bounds_xy'),
                front_bound=target.get('front_bound_m'),rear=target.get('rear_clear'),why=target.get('reason'))
    samples=[r for r in read(run/'trace.json') if tg<=r['t']<=te]
    speed_rows=[dict(t=round(r['t']-tg,3),cmd=r.get('drive_speed_mps'),safe=r.get('safe_speed_mps'),
        ground=r.get('ground_speed_mps'),guard=r.get('safety_reason'),gap=r.get('sampled_gap_m'),
        x=r['x'],y=r['y'],a_x=r.get('actor_x'),a_y=r.get('actor_y')) for r in samples]
    result=dict(scope='diagnostic only; pose/actor truth never reaches controller',
        command_episodes=episodes,unavailable_candidate_reasons=reasons,hold_reasons=holds,
        rejoin_band_counts=band_counts,first_band_examples=band_examples,
        currently_associated_samples=list(per_source.values()),sampled_drive_and_guard=speed_rows)
    with (here/'diagnostic.json').open('x',encoding='utf-8') as f:json.dump(result,f,indent=1,ensure_ascii=False);f.write('\n')
    print('episodes',len(episodes),'blocking reasons',reasons,'fresh target events',len(per_source))
if __name__=='__main__':main(sys.argv[1])
