"""r382 저장 행만으로 보고를 복구한다. 후보/제어기/ROS를 import하거나 실행하지 않는다."""
import argparse
import collections
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import zipfile

A = Path(__file__).resolve().parent
PARENT = A.parent
ROOT = PARENT.parents[3]
PREFIX = PARENT.relative_to(ROOT).as_posix()+'/'
EXPECTED_NAME = 'artifacts/validation/2026-11-13/released_current_r375_v1/results_native_v1/comparison.json'
DIAGNOSTIC = 'current_released_relative_body'


def digest(path):
    value=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):value.update(block)
    return value.hexdigest()


def load_diff():
    spec=importlib.util.spec_from_file_location('r382_recovery_frozen_json_diff',PARENT/'diff_utils.py')
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_jsonl(path):
    content=path.read_bytes()
    assert content.endswith(b'\n'),'incomplete final JSONL line'
    lines=content.splitlines()
    assert all(line.strip() for line in lines),'blank JSONL row'
    return [json.loads(line) for line in lines]


def percentile(values,fraction):
    if not values:return None
    values=sorted(values);position=(len(values)-1)*fraction
    lower=math.floor(position);upper=math.ceil(position)
    return values[lower]+(values[upper]-values[lower])*(position-lower)


def summary(rows):
    values=[r['rebase_wall_ms'] for r in rows]
    assert all(type(v) in (int,float) and math.isfinite(v) and v>=0 for v in values)
    return dict(count=len(values),p50_ms=percentile(values,.5),p95_ms=percentile(values,.95),
        max_ms=max(values) if values else None,above_100ms=sum(v>100 for v in values),above_200ms=sum(v>200 for v in values))


def sensitive_differences(checks):
    # 원 실패는 외부 comparison.kind와 내부 difference.kind를 같은 dict 인수로 전달했다.
    # 두 의미를 별도 키/객체로 보존한다. 덮어쓰거나 삭제하지 않는다.
    return [dict(comparison_kind=row['kind'],index=row['index'],label=row['label'],
                 change_kind=delta['kind'],difference=delta)
        for row in checks for delta in row['differences'] if delta['category'] in (
            'command_or_measured_state','reason_or_selector','permission_or_boolean','time_or_source')]


def trace_records(zipped,name,label):
    records=[]
    with zipped.open(name) as raw:
        for line in io.TextIOWrapper(raw,encoding='utf8'):
            event=json.loads(line)
            if event['kind']=='call' and 'current_rebase' in event['data']:
                current=event['data']['current_rebase'];plan=current['compute']['plan']
                records.append(dict(label=label,output=plan['output'],current=current['input'],expected=current['output']))
    return records


def stored_records(zipped):
    prefix='artifacts/validation/2026-11-13/'
    records=trace_records(zipped,prefix+'relative_body_cad_drive_r371_v1/run1/follow_decisions.jsonl','r371')
    assert len(records)==217
    records.extend(trace_records(zipped,prefix+'released_crawl_drive_r364_v1/run1/follow_decisions.jsonl','r364_non_target'))
    assert len(records)==466
    for selected in json.loads(zipped.read(prefix+'current_input_jetson_r296_v1/selected_inputs.json')):
        row=selected['input']
        records.append(dict(label='holdout_'+selected['source'],output=row['output'],current=row['current_rebase']['input']))
    assert len(records)==572
    return records


def validate_same_arm(record,row,diff):
    before,after=row['before'],row['after']
    retry=after.get(DIAGNOSTIC) or {};applied=retry.get('applied') is True
    assert row['applied'] is applied,'applied metadata differs from output'
    if applied:
        assert record['output'][2]['normal']['selector_mode']=='RELEASED'
        assert record['output'][2]['normal']['selector_choice']=='parallel'
        assert before['speed_mps']==0 and 0<after['speed_mps']<=record['output'][0]<=.10
        assert all(before.get(k)==after.get(k) for k in (
            'steering_rad','reference_s','path_source_s','original_reference_s','path_points'))
        assert before['current_route_proof']['region']==after['current_route_proof']['region']
        assert after['current_route_proof']['accepted'] and not retry['unknown_filled']
        assert after['current_side_presence']=={'left':False,'right':False}
        assert after['maximum_body_corner_lateral_m']<=.01
    else:
        reduced={k:v for k,v in after.items() if k!=DIAGNOSTIC}
        actual=diff.comparison(before,reduced)
        assert actual['existing_comparator_equal'],'same ARM nonapplied changed'
        assert actual==row['same_arm_nonapplied_diff'],'stored same ARM diff incorrect'
    if record['label']!='r371':assert not applied,'non-target applied'
    normal=record['output'][2]['normal']
    return dict(index=row['index'],label=record['label'],reference_s=record['current']['reference_s'],
        mode=normal.get('selector_mode'),choice=normal.get('selector_choice'),applied=applied,
        original_target_mps=record['output'][0],measured_speed_mps=record['current']['speed_mps'],
        before_speed_mps=before['speed_mps'],after_speed_mps=after['speed_mps'],
        before_reason=before['reason'],after_reason=after['reason'],diagnostic=retry)


def validate_rows(timing,checks,records,expected,diff):
    assert len(timing)==589,'expected all 589 timed calls'
    assert len(checks)==821,'expected all 821 comparisons'
    rebuilt=[];negatives=[]
    def check(kind,index,label,reference,actual):
        rebuilt.append(dict(kind=kind,index=index,label=label,**diff.comparison(reference,actual)))
    for index,(row,record) in enumerate(zip(timing[:572],records)):
        assert row['phase']=='stored' and row['index']==index,'stored sequence/index invalid'
        normal=record['output'][2]['normal']
        assert row['label']==record['label']
        assert row['reference_s']==record['current']['reference_s']
        assert row['path_source_s']==normal.get('path_source_s')
        assert row['selector_mode']==normal.get('selector_mode')
        assert row['selector_choice']==normal.get('selector_choice')
        summary_row=validate_same_arm(record,row,diff)
        if record['label']=='r371':
            restored,cap_check=diff.recorded_result(row['before'],record['expected'])
            if cap_check is not None:
                rebuilt.append(dict(kind='x86_parent_cap_precondition',index=index,label=record['label'],**cap_check))
            check('x86_original_current',index,record['label'],record['expected'],restored)
        check('x86_r375_comparison_row',index,record['label'],expected['rows'][index],summary_row)
    for index,(row,reference) in enumerate(zip(timing[572:],expected['counterexamples'])):
        assert row['phase']=='counterexample' and row['index']==index
        assert row['label']=='r371' and row['counterexample']==reference['name']
        before,after=row['before'],row['after']
        assert row['applied'] is False and not (after.get(DIAGNOSTIC) or {}).get('applied')
        reduced={k:v for k,v in after.items() if k!=DIAGNOSTIC}
        actual=diff.comparison(before,reduced)
        assert actual['existing_comparator_equal'] and actual==row['same_arm_nonapplied_diff']
        assert after['speed_mps']==0,'counterexample moved'
        negative=dict(name=reference['name'],stopped=True,baseline_equal=True,reason=after['reason'],speed_mps=after['speed_mps'])
        negatives.append(negative)
        check('x86_counterexample',index,reference['name'],reference,negative)
    assert len(rebuilt)==821
    # 저장된 yes/no/개수만 신뢰하지 않고 모든 기대값·실제값·세부 차이까지 재구성한다.
    assert diff.comparison(checks,rebuilt)['exact_tree_equal'],'comparison rows fail independent reconstruction'
    summary(timing)
    return rebuilt,negatives


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',type=Path,default=PARENT/'remote_result/extracted')
    parser.add_argument('--output',type=Path,default=A/'results')
    args=parser.parse_args()
    source=args.source.resolve();output=args.output.resolve()
    assert output.is_relative_to(A) and not output.exists(),'fresh recovery_r1 result required'
    receipt=json.loads((PARENT/'bundle_receipt.json').read_bytes())
    archive=PARENT/'bundle_sources.zip';manifest_path=PARENT/'source_manifest.json'
    assert digest(archive)==receipt['archive_sha256']
    assert digest(manifest_path)==receipt['manifest_sha256']
    manifest=json.loads(manifest_path.read_bytes())
    original_files=[source/'execution.json',source/'import_preflight.json',source/'import_preflight.log',
        source/'native_build.log',source/'jetson_run.log',source/'jetson/timing_rows.jsonl',source/'jetson/comparison_rows.jsonl',
        archive,manifest_path,PARENT/'run.py',PARENT/'diff_utils.py',PARENT/'query_probe.py',PARENT/'remote_bootstrap.py']
    hashes={str(p.relative_to(ROOT)):digest(p) for p in original_files}
    with zipfile.ZipFile(archive) as zipped:
        assert len(manifest['files'])==278
        for name,expected_hash in manifest['files'].items():
            assert hashlib.sha256(zipped.read(name)).hexdigest()==expected_hash,('archive entry mismatch',name)
        for name in ('run.py','diff_utils.py','query_probe.py','remote_bootstrap.py'):
            assert (PARENT/name).read_bytes()==zipped.read(PREFIX+name),('frozen tool changed',name)
        expected=json.loads(zipped.read(EXPECTED_NAME))
        records=stored_records(zipped)
        run_source=zipped.read(PREFIX+'run.py').decode('utf8')
        assert "sensitive = [dict(kind=r['kind'], index=r['index'], label=r['label'], **d)" in run_source
    diff=load_diff()
    execution=json.loads((source/'execution.json').read_bytes())
    preflight=json.loads((source/'import_preflight.json').read_bytes())
    log=(source/'jetson_run.log').read_text(encoding='utf8')
    assert execution['execution_completed'] is False
    assert execution['error']=="RuntimeError(('jetson_run', 1))"
    assert [s['label'] for s in execution['steps']]==['native_build','import_preflight','jetson_run']
    assert [s['returncode'] for s in execution['steps']]==[0,0,1]
    assert not any(s['forced'] for s in execution['steps'])
    assert execution['changed_sources']==[] and execution['source_count']==278
    assert execution['source_archive_sha256']==receipt['archive_sha256']
    assert preflight['passed'] and preflight['backend']=='native_v1' and preflight['machine']=='aarch64'
    assert preflight['manifest_file_count']==278 and preflight['rebase_calls']==0
    assert len(preflight['imports']['candidate_chain'])==7 and preflight['imports']['native']['abi']==1
    assert log.count('Traceback (most recent call last):')==1
    assert log.rstrip().endswith("TypeError: dict() got multiple values for keyword argument 'kind'")
    assert 'line 157' in log and "sensitive = [dict(kind=r['kind']" in log
    assert not (source/'jetson/report.json').exists(),'original report must remain absent'
    assert not (source/'query_probe').exists(),'probe did not run'
    timing=read_jsonl(source/'jetson/timing_rows.jsonl')
    checks=read_jsonl(source/'jetson/comparison_rows.jsonl')
    rebuilt,negatives=validate_rows(timing,checks,records,expected,diff)
    stored=timing[:572];primary=stored[:217]
    groups=dict(stored_all=summary(stored),r371_all=summary(primary),
        r371_applied=summary([r for r in primary if r['applied']]),r371_unapplied=summary([r for r in primary if not r['applied']]),
        holdout_all=summary(stored[217:]),counterexamples_all=summary(timing[572:]))
    for label in sorted({r['label'] for r in stored}):
        for mode in sorted({str(r['selector_mode']) for r in stored if r['label']==label}):
            groups[label+':'+mode]=summary([r for r in stored if r['label']==label and str(r['selector_mode'])==mode])
    exact=[r for r in rebuilt if not r['exact_tree_equal']]
    mismatches=[r for r in rebuilt if not r['existing_comparator_equal']]
    sensitive=sensitive_differences(rebuilt)
    applied=sum(r['applied'] for r in primary)
    report=dict(schema='r382_offline_recovery_r1',original_pipeline_passed=False,original_pipeline_execution_completed=False,
        recovery_audit_passed=True,original_failure_preserved=True,controller_calls_during_recovery=0,
        computation_batch_completed_evidence=True,same_arm_invariants_reaudited_passed=True,
        x86_existing_comparator_reproduction_passed=not mismatches,x86_exact_tree_reproduction_passed=not exact,
        automatic_adoption_allowed=False,query_probe_executed=False,architecture_cause_confirmed=False,
        original_error="TypeError: dict() got multiple values for keyword argument 'kind'",
        recovery_fix='comparison_kind and change_kind/difference are stored separately; original run.py unchanged',
        stored_calls=572,counterexample_calls=17,timing_rows=589,comparison_rows=821,
        applied_count=applied,x86_expected_applied_count=93,applied_count_matches_x86=applied==93,
        groups=groups,counterexamples=negatives,
        exact_difference_categories=dict(collections.Counter(d['category'] for r in rebuilt for d in r['differences'])),
        beyond_existing_comparator_categories=dict(collections.Counter(d['category'] for r in rebuilt for d in r['differences'] if not d['existing_comparator_equal'])),
        x86_mismatches_by_existing_comparator=[{k:r[k] for k in ('kind','index','label','difference_count','categories')} for r in mismatches],
        command_state_source_permission_exact_differences=sensitive,
        command_state_source_permission_beyond_existing_comparator=[r for r in sensitive if not r['difference']['existing_comparator_equal']],
        original_comparison_rows='remote_result/extracted/jetson/comparison_rows.jsonl',
        original_timing_rows='remote_result/extracted/jetson/timing_rows.jsonl',
        imports=preflight['imports'],python=preflight['python'],machine=preflight['machine'],
        source_input_files=278,changed_sources_reported_by_remote=[],source_archive_sha256=receipt['archive_sha256'],
        original_execution_wall_s=execution['wall_s'],computation_batch_wall_s=None,
        computation_batch_wall_limitation='original began_run/total_wall_s was not persisted; execution wall includes build/preflight',
        measurement='saved original top-level r375 rebase wall times; unchanged candidate/source/order; no rerun',
        command_expiry_evaluated=False,publication_evaluated=False,deadline_permission=False,
        threshold_scope='100/200ms 계산 초과 수이며 안전 허가/명령 만료 통과가 아니다.',
        scope='단일 직렬 저장 계산. ROS/센서 동시부하/명령 전달/실차 증명이 아니며 원 pipeline 실패는 유지한다.')
    audit=dict(schema='r382_offline_row_audit_r1',passed=True,original_hashes=hashes,
        jsonl_complete_lines=True,timing_count=589,comparison_count=821,
        timing_groups=dict(collections.Counter(r['phase']+':'+r['label'] for r in timing)),
        comparison_groups=dict(collections.Counter(r['kind'] for r in checks)),
        exact_input_order_metadata_validated=True,holdout_order='frozen selected_inputs.json order (r279/r282 interleaved)',
        all_821_comparison_rows_rebuilt_exact=True,same_arm_572_and_17_reaudited=True,all_17_counterexamples_stopped=True,
        original_278_archive_entry_hashes_validated=True,remote_changed_sources=[],preflight_passed=True,
        original_exit_codes=[0,0,1],forced_termination=False,
        control_flow_evidence='all 821 comparisons are persisted; source verifies manifest after final comparison; one traceback only at report line157; bootstrap independently reports no source changes',
        limitation='input mutation assertions ran in original frozen loop; raw records do not retain a separate before/after copy of every mutated input',
        query_probe_not_executed=True,original_pipeline_passed=False,controller_calls_during_recovery=0)
    assert all(digest(ROOT/name)==h for name,h in hashes.items()),'original source/result changed during audit'
    output.mkdir()
    for name,value in [('report.json',report),('row_audit.json',audit)]:
        (output/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:report[k] for k in ('recovery_audit_passed','original_pipeline_passed','same_arm_invariants_reaudited_passed','x86_existing_comparator_reproduction_passed','applied_count','groups')},ensure_ascii=False))


if __name__=='__main__':main()
