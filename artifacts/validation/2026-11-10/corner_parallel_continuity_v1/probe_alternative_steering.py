"""같은 센서의 작은 조향 원호를 읽기 전용 대조한다. 실제 명령은 변경하지 않는다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
import arena_autonomy.pass_release_trial as ps


def main():
    out=A/'alternative_steering_probe';out.mkdir(exist_ok=False)
    path=A/'run1/follow_decisions.jsonl';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in
        [path,*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]}
    original=ps.PassReleaseSelector.select;seen=set();rows=[];counts=Counter();groups=Counter()
    def select(self,scene,baseline,parallel,speed,previous,dt,**kw):
        result=original(self,scene,baseline,parallel,speed,previous,dt,**kw)
        group='stopped_motor' if abs(speed)<=ps.mx.STOPPED_SPEED_MPS else 'moving_motor'
        if (result[2].get('selector_reason')!='large_motion_region_unobserved'
                or self.mode!='RELEASED' or scene.source_first_s in seen or groups[group]>=3):return result
        seen.add(scene.source_first_s);groups[group]+=1
        b=baseline[2];path_points=b.get('path_points')
        current=b.get('boundary_from_memory') is False and b.get('path_source_s')==scene.source_first_s
        row=dict(t=scene.t_ref_s,source_s=scene.source_first_s,group=group,measured_speed_mps=speed,
            baseline_target_mps=baseline[0],baseline_steering_rad=baseline[1],previous_steering_rad=previous,
            current_path=current,partial=b.get('partial_observation'),candidates=[])
        for fraction in (1.,.75,.5,.25,0.):
            angle=float(fraction*baseline[1]);target=min(.2,baseline[0])
            region=ps.motion_region(scene,speed,target,angle,previous,dt,self.follow_config,self.lat)
            candidate=dict(fraction=fraction,angle=angle,target=target,
                current_previous_region=region,current_previous_blocked=ps.region_blocked(region,self.config))
            if group=='stopped_motor' and current and path_points:
                # 즉시 발행 가능한 명령이라는 뜻이 아니다. 정지 중 바퀴가 새 각도로
                # 정렬됐다는 별도 가정의 가능성만 분리한다. 실제 speed는 바꾸지 않는다.
                settled=ps.motion_region(scene,speed,target,angle,angle,dt,self.follow_config,self.lat)
                geometry=ps.mx.arc_checks(scene,[],path_points,[angle],settled['distance_m'],
                    self.lat,ps.mx.ExecutionConfig())
                presteer=ps.mx.presteer_support(scene,[],[previous,angle],self.lat,ps.mx.ExecutionConfig())
                candidate.update(settled_hypothetical_region=settled,
                    settled_hypothetical_blocked=ps.region_blocked(settled,self.config),
                    geometry_ok=geometry['geometry_ok'],geometry=geometry['geometry'],presteer=presteer)
            row['candidates'].append(candidate)
        rows.append(freeze(row));return result
    ps.PassReleaseSelector.select=select
    try:
        for event in read_events(path):
            data=event['data']
            if event['kind']=='header':controller,motion=build_controller(data)
            elif event['kind']=='reset':controller.reset()
            elif event['kind']=='call':
                result=freeze(replay_call(controller,data['input'],motion));counts['calls']+=1
                counts['command_mismatch']+=not equal_output(result[:2],data['output'][:2])
    finally:ps.PassReleaseSelector.select=original
    result=dict(scope='read-only lower steering comparisons; presteering is a hypothetical separate step, not a live permission',
        counts=dict(counts),rows=rows,sources=sources,source_unchanged=all(sha(ROOT/p)==h for p,h in sources.items()))
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    summary=[dict(t=r['t'],group=r['group'],current_path=r['current_path'],
        current_candidates=sum(c['current_previous_blocked'] is None for c in r['candidates']),
        hypothetical_candidates=[c['fraction'] for c in r['candidates']
            if c.get('settled_hypothetical_blocked','absent') is None and c.get('geometry_ok')
                and c.get('presteer',{}).get('supported')]) for r in rows]
    print(json.dumps(dict(counts=counts,rows=summary,source_unchanged=result['source_unchanged'])))
    assert counts['command_mismatch']==0 and result['source_unchanged']


if __name__=='__main__':main()
