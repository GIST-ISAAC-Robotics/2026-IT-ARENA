"""한 번의 r470 비교를 집계한다. 새 평가/장치 실행은 하지 않는다."""
import difflib,hashlib,importlib.util,json,math,statistics
from pathlib import Path

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_bytes())

def stats(values):
    ordered=sorted(values);position=(len(values)-1)*.95
    lo,hi=math.floor(position),math.ceil(position)
    return dict(n=len(values),median_ms=statistics.median(values),
        p95_ms=ordered[lo]+(ordered[hi]-ordered[lo])*(position-lo),max_ms=max(values),samples_ms=values)

def main():
    data={v:read(A/'remote_result/results'/f'{v}.json') for v in ('base','stage')}
    receipt=read(A/'remote_result/execution_receipt.json')
    verify=read(A/'remote_result/results/native_verification.json')
    assert receipt['completed'] and verify['passed']
    assert not receipt['source_changes'] and not any(receipt['remaining'].values())
    assert all(x['returncode']==0 and not x['forced'] and not x['residual_group'] for x in receipt['steps'])
    cases=[]
    for key in ('r457:1360','r457:1590','r457:1986','r452:519'):
        versions={}
        for version in data:
            rows=[r for r in data[version]['rows'] if r['id']==key]
            assert len(rows)==3 and all(r['output']==rows[0]['output'] for r in rows)
            names=sorted({c['function'] for r in rows for c in r['helper_calls']})
            versions[version]=dict(wall=stats([r['evaluator_ms'] for r in rows]),
                helpers={name:dict(inclusive_ms=stats([sum(c['ms'] for c in r['helper_calls'] if c['function']==name)
                    for r in rows if r['instrumented']]),calls_per_evaluator=[sum(c['function']==name for c in r['helper_calls'])
                    for r in rows if r['instrumented']]) for name in names},
                output=rows[0]['output'],input_sha256=rows[0]['input_sha256'])
        assert versions['base']['output']==versions['stage']['output']
        cases.append(dict(id=key,versions=versions,whole_output_exact=True,
            median_change_percent=100*(versions['stage']['wall']['median_ms']/versions['base']['wall']['median_ms']-1)))
    manifest=read(A/'source_manifest.json')
    changed=[p for p,h in manifest['files'].items() if sha(A/'bundle'/p)!=h]
    assert not changed
    expected_previous=read(A.parent/'current_structure_cost_r467_v1/optimization_v1/remote_result/results/stage.json')
    previous={r['id']:r['output'] for r in expected_previous['rows'] if r['round']==0}
    assert all(c['versions']['stage']['output']==previous[c['id']] for c in cases)
    report=dict(passed=True,cases=cases,native_verification=verify,changes=read(A/'changes.json'),
        measured_evaluator_calls=24,warmup_calls=2,
        warmups={v:d['warmup'] for v,d in data.items()},
        process_peak_rss_kib={v:d['process_peak_rss_kib'] for v,d in data.items()},
        baseline_policy='r467 optimized',candidate_policy='same policy; occupied-padding numeric predicate native',
        all_output_exact_to_previous_arm=True,local_and_remote_source_changes=[],
        source_count=len(manifest['files']),source_manifest_sha256=sha(A/'source_manifest.json'),
        result_archive_sha256=sha(A/'remote_results.zip'),execution=receipt,
        common_changes_by_this_task=[],packages_installed=False,sensors_opened=False,
        timings_scope='Evaluator only; wrapper round1/2 included; constructor/request/hash/I/O/IPC excluded',
        p95_scope='linear interpolation of three samples per case; not population or deadline guarantee',
        native_occupancy_timing_nested_inside_helper=True)
    (A/'summary.json').write_text(json.dumps(report,indent=2))
    patches=[]
    for row in report['changes']:
        before=A/'bundle/base'/row['path'];after=A/'stage'/row['path']
        assert sha(after)==row['after']
        patches.extend(difflib.unified_diff(before.read_text(encoding='utf8').splitlines(True) if before.exists() else [],
            after.read_text(encoding='utf8').splitlines(True),
            fromfile='a/'+row['path'] if before.exists() else '/dev/null',tofile='b/'+row['path']))
    (A/'candidate.patch').write_text(''.join(patches),encoding='utf8',newline='\n')
    resource=dict(returned=True,remaining=receipt['remaining'],source_changes=[],
        pids=[x['pid'] for x in receipt['steps']],forced_kills=0,
        measured_evaluator_calls=24,warmup_calls=2,additional_full_evaluator_calls=0,
        native_builds=1,packages_installed=False,devices_opened=False)
    (A/'resource_return.json').write_text(json.dumps(resource,indent=2))
    print(json.dumps(dict(cases=[dict(id=c['id'],before=c['versions']['base']['wall'],after=c['versions']['stage']['wall'],
        change_percent=c['median_change_percent']) for c in cases],kernel_cases=len(verify['kernel_checks']),
        helper_cases=len(verify['helper_checks']),resource=resource)))

if __name__=='__main__':main()
