"""종료된 시험의 기존 판정·선택 상태 요약. 정답은 평가에만 사용한다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def read(p):return json.loads(p.read_text(encoding='utf-8'))


def report_summary(p):
    r=read(p/'report.json');e=r['evaluation'];actors=r.get('additional_actors',{})
    return {'passed':r['passed'],'run_valid':r['run_valid'],'source_unchanged':r['source_unchanged'],
        'progress':e['progress'],'case_metrics':e['case_metrics'],'primary_gap':e['gap'],
        'body_road':e['road'],'gates':r['gates'],'stop_checks':r['stop_checks'],
        'shutdown_clean':r['shutdown_clean'],'remaining_test_pids':r['remaining_test_pids'],
        'additional_actors':{n:{k:v for k,v in a.items() if k in (
            'gates','min_sampled_gap_m','min_swept_lower_bound_m','encounter')} for n,a in actors.get('actors',{}).items()},
        'additional_ego_contact_events':actors.get('combined_contacts',{}).get('ego_events'),
        'selector_diagnostics':r.get('pass_release_trial')}


def decisions(path):
    rows=[];resets=[]
    for line in path.read_text(encoding='utf-8').splitlines():
        event=json.loads(line)
        if event['kind']=='reset':resets.append(event);continue
        if event['kind']!='call':continue
        data=event['data'];v,steer,d=data['output']
        rows.append({'t':data['input']['reference_s'],'speed':v,'steer':steer,
            'mode':d.get('selector_mode',d.get('selector_mode_before')), 'choice':d.get('selector_choice'),
            'reason':d.get('selector_reason'),'controller_reason':d.get('reason'),
            'region':d.get('selector_region'),'sides':d.get('selector_sides')})
    transitions=[];old=None;choices=[];old_choice=None
    for row in rows:
        if row['mode']!=old:transitions.append(row);old=row['mode']
        if row['choice']!=old_choice:choices.append(row);old_choice=row['choice']
    return {'calls':len(rows),'resets':len(resets),'modes':dict(Counter(r['mode'] for r in rows)),
        'choices':dict(Counter(r['choice'] for r in rows)), 'reasons':dict(Counter(r['reason'] for r in rows)),
        'controller_reasons':dict(Counter(r['controller_reason'] for r in rows)),
        'mode_transitions':transitions,'choice_transitions_count':len(choices),
        'maximum_steering_step_rad':max((abs(b['steer']-a['steer']) for a,b in zip(rows,rows[1:])),default=0),
        'maximum_target_mps':max((r['speed'] for r in rows),default=0),'last':rows[-1] if rows else None}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path)
    parser.add_argument('--compare',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    files=[args.run/'report.json',args.run/'follow_decisions.jsonl',args.compare/'report.json',Path(__file__)]
    result={'scope':'completed simulation report and recorded decisions; not physical validation',
        'sources':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        'before':report_summary(args.compare),'after':report_summary(args.run),
        'decisions':decisions(args.run/'follow_decisions.jsonl')}
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'progress':result['after']['progress'],'decision_modes':result['decisions']['modes'],
                     'choices':result['decisions']['choices'],'stop':result['after']['stop_checks']},ensure_ascii=False))
