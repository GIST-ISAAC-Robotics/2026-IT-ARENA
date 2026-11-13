"""회수된 원 결과만 집계한다. 제어 함수·remote 실행 없음."""
from pathlib import Path
import hashlib,json,math,statistics
BASE=Path(__file__).resolve().parent; REPO=BASE.parents[3]
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def numeric_delta(a,b):
    if isinstance(a,dict) and isinstance(b,dict):
        assert set(a)==set(b);return max((numeric_delta(a[k],b[k]) for k in a),default=0.)
    if isinstance(a,list) and isinstance(b,list):
        assert len(a)==len(b);return max((numeric_delta(x,y) for x,y in zip(a,b)),default=0.)
    if type(a) is float or type(b) is float:
        assert isinstance(a,(int,float)) and isinstance(b,(int,float));return abs(a-b)
    assert a==b;return 0.
def stats(v):
    x=sorted(v);return dict(n=len(x),minimum=min(x),median=statistics.median(x),p95=x[math.ceil(.95*len(x))-1],maximum=max(x))
def main():
    report=json.loads((BASE/'remote_result/cost/report.json').read_bytes())
    receipt=json.loads((BASE/'remote_result/execution.json').read_bytes())
    rows=[json.loads(x) for x in (BASE/'remote_result/cost/rows.jsonl').open()]
    expected={r['seq']:r for r in json.loads((BASE/'bundle/inputs/expected.json').read_bytes())}
    manifest=json.loads((BASE/'source_manifest.json').read_bytes())
    source_changes=[p['source'] for p in manifest['provenance'].values() if digest(REPO/p['source'])!=p['sha256']]
    guards=[r['candidate_output'][2]['selector_guard_candidate'] for r in rows]
    result=dict(passed=report['passed'],rows=len(rows),original_matches=report['original_matches'],
        candidate_accepted=report['candidate_accepted'],current_dispatch_supported=report['current_supported'],
        original_guard_calls=sum(len(r['original_calls']) for r in rows),
        additional_guard_calls=sum(len(r.get('candidate_calls',[])) for r in rows),
        signed_measured_mps=stats([r['signed_speed_mps'] for r in rows]),
        masked_tail_extent_m=stats([g['region']['masked_tail_extent_m'] for g in guards]),
        unobserved_counts=sorted(set(g['region']['unobserved'] for g in guards)),
        front_mask_counts=sorted(set(g['region']['masked_front_samples'] for g in guards)),
        side_state_preserved=all(r['selector_state_unchanged'] for r in rows),
        integrated_max_float_delta=max(numeric_delta(r['candidate_output'],expected[r['seq']]['adapter_result']) for r in rows),
        integrated_integer_and_structure_exact=True,costs_ms=report['costs_ms'],
        paired_addition_fraction=stats([r['adapter_ns']/r['original_ns'] for r in rows]),
        current_dispatch_check_is_execution=False,full_plan_replayed=False,
        source_changes=source_changes,remote_source_changes=receipt['source_changes'],
        batch_exit=receipt['step']['exit_code'],forced=receipt['step']['forced'],
        residual_group=receipt['step']['residual_group'],owned_remaining=receipt['owned_remaining'],
        remote_root='/home/q/it-arena-r454-a',jetson_released=True,
        archive_sha256=digest(BASE/'bundle_sources.zip'),result_archive_sha256=digest(BASE/'remote_results.zip'),
        native_sha256=report['native_sha256'],wsl_used=False,devices_opened=False,settings_changed=False)
    assert not source_changes and result['original_guard_calls']==312 and result['additional_guard_calls']==104
    (BASE/'summary.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('costs_ms',)},ensure_ascii=False))
if __name__=='__main__':main()
