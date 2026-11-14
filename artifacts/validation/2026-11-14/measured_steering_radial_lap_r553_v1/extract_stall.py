"""종료 후 정체 전후 대표 입력만 발췌한다. 제어기 재생/원 결과 수정 없음."""
import hashlib,json
from pathlib import Path
A=Path(__file__).resolve().parent;R=A/'run1';OUT=A/'stall_extract'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
    before=sha(R/'report.json');OUT.mkdir(exist_ok=True);assert not list(OUT.iterdir());calls=[];pubs=[];plans={};pending=None;header=None
    for line,text in enumerate((R/'follow_decisions.jsonl').open(encoding='utf-8'),1):
        row=json.loads(text);d=row.get('data') or {};kind=row['kind'];item=dict(source_line=line,record=row)
        if kind=='header':header=item
        elif kind in ('runtime_plan_submitted','runtime_plan_completed'):plans.setdefault((d.get('generation'),d.get('sequence')), {})[kind]=item
        elif kind=='call':calls.append(item);pending=item
        elif kind=='route_publication':pubs.append((item,pending));pending=None
    positive_pub,positive_call=next((p,c) for p,c in reversed(pubs) if p['record']['data'].get('speed_mps',0)>0)
    after=[c for c in calls if c['record']['seq']>positive_call['record']['seq']]
    first_zero=next((c for c in after if c['record']['data']['output'][0]==0 and c['record']['data']['delivery']['kind']=='discarded'),None)
    accepted_zero=next((c for c in after if c['record']['data']['output'][0]==0 and c['record']['data']['delivery']['kind']=='candidate'),None)
    zero_pub,zero_pub_call=next((p,c) for p,c in pubs if p['record']['seq']>positive_pub['record']['seq'] and p['record']['data'].get('speed_mps')==0)
    summary={};targets=[]
    for name,item in [('last_positive',positive_call),('first_zero_discarded',first_zero),('first_zero_candidate',accepted_zero),('first_zero_publication_call',zero_pub_call)]:
        if item is None:continue
        d=item['record']['data'];meta=d['output'][2];normal=meta.get('normal') or {};compute=d.get('compute') or {};reference=d['input']['reference_s'];targets.append(reference)
        write(OUT/(name+'_call.json'),item);write(OUT/(name+'_current_input.json'),d['input']);write(OUT/(name+'_compute_input.json'),compute.get('input'))
        associated=plans.get((compute.get('generation'),compute.get('sequence')),{})
        write(OUT/(name+'_plan.json'),associated)
        summary[name]=dict(call_seq=item['record']['seq'],source_line=item['source_line'],reference_s=reference,
            output_speed_mps=d['output'][0],output_steering_rad=d['output'][1],previous_steering_rad=d['input'].get('previous_steering_rad'),
            delivery=d['delivery'],handoff_state=meta.get('handoff_state'),reason=normal.get('reason'),baseline_reason=normal.get('baseline_reason'),
            underlying_reason=normal.get('underlying_reason'),path_wall=normal.get('path_wall'),path_source_s=normal.get('path_source_s'),
            path_points_count=len(normal.get('path_points') or []),current_wall_reacquire=normal.get('current_wall_reacquire'),
            input_file=name+'_current_input.json',compute_input_file=name+'_compute_input.json',call_file=name+'_call.json',plan_file=name+'_plan.json')
    write(OUT/'last_positive_publication.json',positive_pub);write(OUT/'first_zero_publication.json',zero_pub);write(OUT/'header.json',header)
    nearest={ref:None for ref in targets}
    with (R/'safety_status.jsonl').open(encoding='utf-8') as stream:
        for line,text in enumerate(stream,1):
            row=json.loads(text);ref=row['data'].get('reference_s')
            if ref is None:continue
            for target in nearest:
                if nearest[target] is None or abs(ref-target)<nearest[target][0]:nearest[target]=(abs(ref-target),dict(source_line=line,record=row))
    for name,item in summary.items():
        raw=nearest[item['reference_s']][1];write(OUT/(name+'_guard.json'),raw)
        item['guard']=dict(file=name+'_guard.json',reference_s=raw['record']['data']['reference_s'],reason=raw['record']['data'].get('reason'),safe_speed_mps=raw['record']['data'].get('safe_speed_mps'))
    traj=[json.loads(l) for l in (R/'trajectory.jsonl').read_text().splitlines()]
    cutoff=positive_pub['record']['data']['reference_s'];zero=next(r for r in traj if r['t']>=cutoff and r['drive_speed_mps']==0)
    result=dict(scope='Stored evidence extraction only; terminal sustained stop, not initial warmup zero; discarded delivery and accepted zero plan separated',
        source_trace='run1/follow_decisions.jsonl',source_trace_sha256=sha(R/'follow_decisions.jsonl'),original_report_sha256=before,
        last_positive_publication=positive_pub['record']['data'],transition=summary,first_zero_drive_trajectory=zero,
        first_zero_discarded_present=first_zero is not None,first_zero_candidate_present=accepted_zero is not None,
        first_zero_publication=zero_pub['record']['data'],later_call_count=len(after),later_positive_candidate_count=sum(c['record']['data']['output'][0]>0 and c['record']['data']['delivery']['kind']=='candidate' for c in after),discarded_zero_scope='Calls after final positive route publication only; missing is not synthesized',
        all_later_publications_positive=False,source_report_unchanged=sha(R/'report.json')==before)
    write(OUT/'transition.json',result);assert result['source_report_unchanged']
    print(json.dumps(dict(path=str(OUT/'transition.json'),transitions={k:dict(seq=v['call_seq'],ref=v['reference_s'],speed=v['output_speed_mps'],reason=v['reason'],baseline=v['baseline_reason'],delivery=v['delivery']['kind']) for k,v in summary.items()}),ensure_ascii=False))
if __name__=='__main__':main()
