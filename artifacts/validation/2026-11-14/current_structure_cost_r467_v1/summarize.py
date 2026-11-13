"""원 측정과 후속 최적화의 출력·비용·회수 근거를 재계산 없이 집계한다."""
import difflib
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import statistics

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
O=A/'optimization_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
module_path=ROOT/'artifacts/validation/2026-11-13/runtime_cost_r411_v1/diff_utils.py'
spec=importlib.util.spec_from_file_location('r467_diff',module_path)
diff=importlib.util.module_from_spec(spec);spec.loader.exec_module(diff)
read=lambda p:json.loads(p.read_bytes())

def stats(values):
    ordered=sorted(values);index=(len(values)-1)*.95
    low,high=math.floor(index),math.ceil(index)
    p95=ordered[low]+(ordered[high]-ordered[low])*(index-low)
    return dict(n=len(values),median_ms=statistics.median(values),p95_ms=p95,max_ms=max(values),
        samples_ms=values,p95_method='linear interpolation of these few saved samples; not a population guarantee')

def compare_round(folder):
    data={v:read(folder/'remote_result/results'/f'{v}.json') for v in ('base','stage')}
    cases=[]
    for key in ('r457:1360','r457:1590','r457:1986','r452:519'):
        versions={}
        for version in data:
            rows=[r for r in data[version]['rows'] if r['id']==key]
            assert len(rows)==3 and all(r['output']==rows[0]['output'] for r in rows)
            functions=sorted({call['function'] for row in rows for call in row['helper_calls']})
            helpers={name:dict(per_evaluator_total=stats([sum(c['ms'] for c in r['helper_calls'] if c['function']==name)
                    for r in rows if r['instrumented']]),
                calls_per_evaluator=[sum(c['function']==name for c in r['helper_calls']) for r in rows if r['instrumented']])
                for name in functions}
            versions[version]=dict(wall=stats([r['evaluator_ms'] for r in rows]),
                helpers=helpers,output=rows[0]['output'],input_sha256=rows[0]['input_sha256'],
                all_rounds_exact=True,instrumentation_output_exact=True)
        before,after=(versions[v]['output'] for v in ('base','stage'))
        command_fields=('speed_mps','steering_rad','path_points','reference_s',
            'independent_guard_required','runtime_authorized')
        comparisons=diff.comparison(before,after)
        cases.append(dict(id=key,versions=versions,all_output_comparison=comparisons,
            unchanged_path_steering_time_flags=all(before.get(k)==after.get(k) for k in command_fields if k!='speed_mps'),
            command_before={k:before.get(k) for k in (*command_fields,'reason')},
            command_after={k:after.get(k) for k in (*command_fields,'reason')},
            median_change_percent=100*(versions['stage']['wall']['median_ms']/versions['base']['wall']['median_ms']-1)))
    receipt=read(folder/'remote_result/execution_receipt.json')
    assert receipt['completed'] and not receipt['source_changes'] and not any(receipt['remaining'].values())
    assert all(s['returncode']==0 and not s['forced'] and not s['residual_group'] for s in receipt['steps'])
    return dict(cases=cases,measured_calls=sum(v['measured_calls'] for v in data.values()),
        warmup_calls=sum(v['warmup_calls'] for v in data.values()),
        warmups={v:d['warmup'] for v,d in data.items()},constructors_ms={v:d['constructor_ms'] for v,d in data.items()},
        process_peak_rss_kib={v:d.get('process_peak_rss_kib') for v,d in data.items()},
        execution_receipt_sha256=sha(folder/'remote_result/execution_receipt.json'),
        result_archive_sha256=sha(folder/'remote_results.zip'),steps=receipt['steps'],
        source_changes=[],remaining=receipt['remaining'])

def main():
    initial=compare_round(A);optimized=compare_round(O)
    inputs=read(A/'bundle/inputs.json')
    cross_platform=[]
    for record in inputs['records']:
        case=next(c for c in initial['cases'] if c['id']==record['id'])
        cross_platform.append(dict(id=record['id'],comparison=diff.comparison(record['expected'],case['versions']['base']['output'])))
    assert all(c['unchanged_path_steering_time_flags'] for c in initial['cases'])
    assert all(c['all_output_comparison']['exact_tree_equal'] for c in optimized['cases'])
    assert all(c['all_output_comparison']['exact_tree_equal'] for c in initial['cases'] if c['id'] in ('r457:1360','r452:519'))
    original_manifest=read(A/'source_manifest.json')
    original_sources_changed=[p for p,h in original_manifest['original_sources'].items() if sha(ROOT/p)!=h]
    assert not original_sources_changed
    source_checks={}
    for folder,bundle in ((A,A/'bundle'),(O,O/'bundle_v2')):
        manifest=read(folder/'source_manifest.json')
        changed=[p for p,h in manifest['files'].items() if sha(bundle/p)!=h]
        assert not changed
        source_checks[folder.name]=dict(count=len(manifest['files']),changed=changed,
            manifest_sha256=sha(folder/'source_manifest.json'),input_sha256=sha(bundle/'inputs.json'))
    result=dict(initial_r463_comparison=initial,optimization_comparison=optimized,
        x86_original_differences=cross_platform,source_checks=source_checks,
        original_sources_changed=original_sources_changed,local_verification=read(O/'local_verification.json'),
        source_change=read(O/'changes.json'),common_files_changed_by_this_task=[],
        physical_sensor_input=False,hardware_activation=False,ros_realtime_verified=False,
        measured_evaluator_calls=48,warmup_calls=4,full_plan_replays=0,
        timings_include='Evaluator restore/scene/current/freeze plus wrapper bookkeeping in rounds1/2',
        timings_exclude='import, constructors, current_request creation, hashing, output save, SSH, IPC',
        iteration_limit_respected=True,jetson_resource_returned=True)
    (A/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    resource=dict(resource='Jetson CPU calculation',returned=True,remaining=optimized['remaining'],
        pids=[r['pid'] for group in (initial,optimized) for r in group['steps']],
        forced_kills=0,measured_evaluator_calls=48,warmup_calls=4,source_changes=[],
        device_ports_opened=False,settings_changed=False)
    (A/'resource_return.json').write_text(json.dumps(resource,indent=2))
    path=Path('src/arena_autonomy/arena_autonomy/known_structure.py')
    patch=''.join(difflib.unified_diff((A/'bundle/stage'/path).read_text(encoding='utf8').splitlines(True),
        (O/'stage'/path).read_text(encoding='utf8').splitlines(True),fromfile='a/'+path.as_posix(),tofile='b/'+path.as_posix()))
    (O/'candidate.patch').write_text(patch,encoding='utf8',newline='\n')
    print(json.dumps(dict(comparisons=[dict(id=c['id'],before=c['versions']['base']['wall'],after=c['versions']['stage']['wall'],
        change_percent=c['median_change_percent']) for c in optimized['cases']],source_checks=source_checks,
        resource_return=resource,x86_existing_comparator=[dict(id=c['id'],equal=c['comparison']['existing_comparator_equal'],
            categories=c['comparison']['categories']) for c in cross_platform])))


if __name__=='__main__':main()
