"""회수 자료만 요약한다. Evaluator·장치·ROS를 실행하지 않는다."""
from pathlib import Path
from collections import Counter
from datetime import datetime,timezone
import hashlib,json,statistics
BASE=Path(__file__).resolve().parent;REPO=BASE.parents[3]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    report=json.loads((BASE/'remote_result/cost/report.json').read_bytes())
    rows={c:list(map(json.loads,(BASE/'remote_result/cost'/c/'rows.jsonl').open())) for c in ('simulation','estimated')}
    summary={k:v for k,v in report.items() if k!='contracts'}
    summary['contracts']={c:{k:v for k,v in report['contracts'][c].items() if k not in ('imports',)} for c in rows}
    summary['slow_estimated']=[dict(key=r['key'],ms=r['evaluator_ms'],counts=r['diagnostic_counts'])
        for r in rows['estimated'] if r['evaluator_ms']>100]
    summary['representatives']=[dict(key=a['key'],role=a['role'],simulation_ms=a['evaluator_ms'],estimated_ms=b['evaluator_ms'],
        speed_mps=a['output']['speed_mps'],reason=a['output']['reason'],
        same_arm_entire_output_exact=a['output']==b['output']) for a,b in zip(rows['simulation'][-3:],rows['estimated'][-3:])]
    summary['x86_failed_keys']=[r['key'] for r in rows['simulation'] if not r['x86_comparison']['existing_comparator_equal']]
    summary['x86_failed_difference_kinds']=dict(Counter(d['category'] for r in rows['simulation']
        for d in r['x86_comparison']['differences'] if not d['existing_comparator_equal']))
    for c,items in rows.items():
        summary['contracts'][c]['runtime_authorized_true']=sum(r['output'].get('runtime_authorized') is True for r in items)
        summary['contracts'][c]['independent_guard_required_true']=sum(r['output'].get('independent_guard_required') is True for r in items)
        summary['contracts'][c]['crop_reasons']=dict(Counter((r['output'].get('current_crop_component_retry') or {}).get('reason')
            for r in items if 'current_crop_component_retry' in r['output']))
    estimated_trials=[r['output']['current_crop_component_retry'] for r in rows['estimated']
        if 'current_crop_component_retry' in r['output']]
    summary['estimated_diagnostics_preserved']=dict(count=len(estimated_trials),
        all_timing_verified_false=all(t.get('timing_verified') is False for t in estimated_trials),
        all_host_estimated=all(t.get('timing_policy')=='host_estimated_v1' for t in estimated_trials),
        all_single_target=all(t['attempted_targets']==1 for t in estimated_trials))
    manifest=json.loads((BASE/'source_manifest.json').read_bytes())
    changed=[n for n,h in manifest['provenance'].items() if sha(REPO/h['source'])!=h['sha256']]
    summary['local_provenance_changes']=changed
    summary['node_check']=json.loads((BASE/'remote_node_v2/report.json').read_bytes())
    summary['node_check'].pop('selected',None)
    summary['node_receipt']=json.loads((BASE/'remote_node_v2/receipt.json').read_bytes())
    summary['final_audit']=json.loads((BASE/'final_audit.log').read_text(encoding='utf8').strip())
    summary['recall_sha256']=dict(cost_zip=sha(BASE/'remote_results.zip'),nodes_zip=sha(BASE/'remote_node_v2_results.zip'))
    summary['completed_at_utc']=datetime.now(timezone.utc).isoformat()
    assert not changed and summary['node_check']['passed'] and summary['same_arm_invariants_passed']
    assert summary['estimated_diagnostics_preserved']['all_timing_verified_false']
    assert summary['estimated_diagnostics_preserved']['all_single_target']
    assert summary['representatives'][0]['same_arm_entire_output_exact']
    (BASE/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:summary[k] for k in ('estimated_diagnostics_preserved','x86_failed_difference_kinds',
        'representatives','slow_estimated','local_provenance_changes')},indent=2))
if __name__=='__main__':main()
