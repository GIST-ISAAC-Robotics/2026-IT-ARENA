"""현재 선택형 양보의 실제 기록·종료·소스를 사후 검토한다."""
from collections import Counter
from hashlib import sha256
from pathlib import Path
import json,sys,zipfile

ROOT=Path(__file__).resolve().parents[4]

def review(run):
    r=json.loads((run/'report.json').read_bytes())
    green=r['case_window']['green_sim_s'];end=green+r['case_window']['duration_s']
    transitions=[];phase_events=[];renewals=[];states=Counter();sources={};seen={}
    with (run/'follow_decisions.jsonl').open('rb') as f:
        digest=sha256();footer=None;header=None
        for seq,line in enumerate(f):
            digest.update(line);e=json.loads(line);assert seq==e['seq'] and footer is None
            if e['kind']=='header':header=e['data']
            if e['kind']=='footer':footer=e['data']
            if e['kind']!='call':continue
            inp=e['data']['input'];t=inp['reference_s']
            if not green<=t<=end:continue
            v,a,q=e['data']['output'];state=q.get('handoff_state');states[state]+=1
            if not transitions or transitions[-1]['state']!=state:
                transitions.append(dict(t=t-green,state=state))
            phase=(q.get('bypass') or {}).get('entry_phase')
            if phase is not None and (not phase_events or phase_events[-1]['phase']!=phase):
                phase_events.append(dict(t=t-green,phase=phase))
            y=(q.get('relative_yield') or {}).get('current_observer') or {}
            if y.get('progress_budget_renewed'):
                renewals.append(dict(t=t-green,renewals=y.get('progress_budget_renewals'),
                    total_yield_s=y.get('total_yield_s'),mark_m=y.get('progress_mark_m')))
    assert digest.hexdigest()==r['follow_decisions']['sha256']
    assert footer['written']==seq and not any(footer[k] for k in ('failed','dropped','capture_errors','pending'))
    assert header['execution_config']['relative_yield_policy']=='observed_progress_yield_v3'
    with zipfile.ZipFile(run/'source_snapshot.zip') as z:
        for n in ['src/arena_autonomy/arena_autonomy/bypass_yield_progress.py','src/arena_autonomy/arena_autonomy/bypass_handoff.py',
                  'scripts/validate_g1_traffic.py','scripts/g1_handoff_eval.py','src/arena_bringup/launch/simulation.launch.py']:
            actual=sha256(z.read(n)).hexdigest()
            sources[n]=dict(snapshot_matches_report=actual==r['source_sha256'][n],
                snapshot_matches_current=actual==sha256((ROOT/n).read_bytes()).hexdigest())
    assert all(all(x.values()) for x in sources.values())
    return dict(run=str(run),valid=r['run_valid'],source_unchanged=r['source_unchanged'],clean=r['shutdown_clean'],
      raw_failed_gates=[k for k,v in r['gates'].items() if not v],stop=r['stop_checks'],progress=r['evaluation']['progress'],
      gap=r['evaluation']['gap'],states=dict(states),transitions=transitions,phase_events=phase_events,renewals=renewals,
      source_audit=sources,scope='실행 안 상대차 자세는 평가 전용. 15초 종료 밖 복귀/정체를 추정하지 않음.')
if __name__=='__main__':
    r=review(Path(sys.argv[1]))
    with Path(sys.argv[2]).open('x',encoding='utf-8') as f:json.dump(r,f,ensure_ascii=False,indent=2)
    print(json.dumps({k:r[k] for k in ['valid','source_unchanged','clean','raw_failed_gates','transitions','phase_events']},ensure_ascii=False))
