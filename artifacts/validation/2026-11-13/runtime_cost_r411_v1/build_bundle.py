"""원 r406 저장값 선택 + 현행 common 정적 import 폐포. 실제 제어기 import/실행 없음."""
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]


def sha(data):return hashlib.sha256(data).hexdigest()
def encoded(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def pretty(value):return (json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode('utf8')


def main():
    assert not (HERE/'bundle_sources.zip').exists() and not (HERE/'source_manifest.json').exists()
    lineage=json.loads((HERE/'lineage.json').read_bytes())
    for row in lineage['source_package']+lineage['boundary_sources']:
        assert sha((ROOT/row['path']).read_bytes())==row['sha256'],('source changed after approved plan',row['path'])
    trace=ROOT/lineage['trace']['path'];assert sha(trace.read_bytes())==lineage['trace']['sha256']
    original=[];currents=[];seen={};header=None;factory=None;reason=None
    for line in trace.open(encoding='utf8'):
        row=json.loads(line);data=row['data']
        if row['kind']=='header':header=data
        elif row['kind']=='runtime_factory_policy':factory=data
        elif row['kind']=='reset':reason=data.get('reason')
        elif row['kind']=='call':
            compute=data['compute'];key=(compute['generation'],compute['sequence'])
            assert data['input']==compute['input']
            identity=(sha(encoded(data['input'])),sha(encoded(data['output'])))
            if key in seen:assert seen[key]==identity
            else:
                seen[key]=identity
                original.append(dict(index=len(original),trace_seq=row['seq'],generation=key[0],sequence=key[1],
                    reset_reason=reason,data=data['input'],data_sha256=identity[0],expected=data['output'],
                    selector_mode=data['output'][2].get('normal',{}).get('selector_mode'),
                    source=dict(reference_s=data['input']['reference_s'],epoch=data['input']['epoch'],
                        scan_source_epoch=data['input']['scan']['measurements']['source_epoch'])))
            current=data.get('current_rebase')
            if current:
                cc=current['compute'];plan=cc['plan'];snapshot=current['input'];prior=plan['input']
                assert snapshot==cc['input'] and cc['route_check']==plan['route_check']
                # 공통 runtime_dual_process.plan_reference의 원 계약을 그대로 구성한다.
                reference=dict(reference_s=prior['reference_s'],epoch=prior['epoch'],side=prior['side'],
                    previous_steering_rad=prior['previous_steering_rad'],scan=dict(representation=prior['scan']['representation'],
                    measurements=dict(source_epoch=prior['scan']['measurements']['source_epoch'])))
                owned=dict(output=plan['output'],submitted=reference,current=snapshot,retained=cc['route_check']['retained'])
                currents.append(dict(index=len(currents),trace_seq=row['seq'],generation=cc['generation'],sequence=cc['sequence'],
                    reset_reason=None,data=owned,data_sha256=sha(encoded(owned)),expected=current['output'],
                    selector_mode=plan['output'][2].get('normal',{}).get('selector_mode'),
                    source=dict(reference_s=snapshot['reference_s'],epoch=snapshot['epoch'],
                        scan_source_epoch=snapshot['scan']['measurements']['source_epoch'],
                        plan_generation=plan['generation'],plan_sequence=plan['sequence'],retained=owned['retained'])))
    plans=[r for r in original if r['sequence']<74]
    assert len(original)==254 and len(plans)==37 and len(currents)==153
    assert len(seen)==254 and plans[0]['reset_reason']=='WAIT_GREEN'
    assert factory['policy']=='cad_runtime_r377_v1' and factory['motion_time_policy']=='verified_only'
    assert factory['current_corner_budget']=='r396_v1' and factory['motion_timing_verified'] is True
    for record in plans+currents:
        snapshot=record['data'] if 'scan' in record['data'] else record['data']['current']
        raw=snapshot['scan']['measurements']
        assert snapshot['timing_verified'] is True and snapshot.get('timing_policy','verified_only')=='verified_only'
        assert raw['time_basis']=='simulation_acquisition' and raw['source_frame_id']=='laser_frame'
    selected=dict(header,runtime_policy=factory['policy'],runtime_time_policy=factory['motion_time_policy'],
        numeric_execution='native_v1',current_corner_budget='off')
    caps=[r['index'] for r in currents if 'new_plan_deceleration_cap' in r['expected']]
    assert len(caps)==23
    selection=dict(schema='r411_input_contract_v1',trace=lineage['trace'],plan_count=37,current_count=153,
        full_calls=380,smoke_calls=80,smoke_current_indices=[0,42,137],plan_prefix_sequence_limit_exclusive=74,
        excluded_unique_plans=217,sequence_coverage=lineage['sequence_coverage'],recorded_candidate_applied_indices=[137],
        parent_cap_indices=caps,original_current_positive=145,original_current_zero=8,
        header_added_fields=dict(runtime_policy=factory['policy'],runtime_time_policy=factory['motion_time_policy'],
            numeric_execution='explicit native_v1 for the registered runtime'),
        header_arm_difference=['current_corner_budget','current_corner_environment'],
        actual_current_input_extra_field='retained preserved from original route_check',
        original_recorded_source_timestamps_preserved=True,real_sensor_inputs=False)
    inputs={'header.json':pretty(selected),'raw_header.json':pretty(header),'runtime_factory_policy.json':pretty(factory),
        'selection.json':pretty(selection),'plans.jsonl':b''.join(encoded(r)+b'\n' for r in plans),
        'currents.jsonl':b''.join(encoded(r)+b'\n' for r in currents)}
    names=('run.py','support.py','timed_evaluator.py','diff_utils.py','preflight.py','remote_bootstrap.py','test_helpers.py')
    overlay={'r411/'+name:HERE/name for name in names}
    class Sources:
        def __contains__(self,name):return name in overlay or (ROOT/name).is_file()
        def __getitem__(self,name):return overlay.get(name,ROOT/name).read_bytes()
    spec=importlib.util.spec_from_file_location('r411_static_closure',ROOT/'scripts/python_dependency_closure.py')
    closure_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(closure_module)
    entries=list(overlay)+[row['path'] for row in lineage['source_package']]
    closure=closure_module.collect(entries,roots=closure_module.PACKAGE_ROOTS+('r411',),members=Sources())
    members={n:overlay.get(n,ROOT/n).read_bytes() for n in closure['files']}
    assert not any('r410' in n or n.startswith('artifacts/') for n in members)
    for name,data in inputs.items():members['r411/inputs/'+name]=data
    members['r411/lineage.json']=(HERE/'lineage.json').read_bytes()
    for name,data in members.items():
        if name.endswith('.py'):ast.parse(data,filename=name)
    # 이미 승인된 원 source가 묶음 생성 도중 바뀌지 않았는지 다시 확인한다.
    for row in lineage['source_package']+lineage['boundary_sources']:
        assert sha((ROOT/row['path']).read_bytes())==row['sha256']
    assert sha(trace.read_bytes())==lineage['trace']['sha256']
    input_contract=sha(encoded({n:sha(d) for n,d in sorted(inputs.items())}))
    manifest=dict(schema='runtime_cost_r411_source_manifest_v1',files={n:sha(d) for n,d in sorted(members.items())},
        input_contract_sha256=input_contract,trace_sha256=lineage['trace']['sha256'],
        package_files=23,original_algorithm_files=17,closure=closure,common_changes=0,r410_candidate_included=False,
        full_calls=380,smoke_calls=80)
    (HERE/'inputs').mkdir()
    for name,data in inputs.items():(HERE/'inputs'/name).write_bytes(data)
    (HERE/'source_manifest.json').write_bytes(pretty(manifest))
    with zipfile.ZipFile(HERE/'bundle_sources.zip','x',zipfile.ZIP_DEFLATED) as out:
        for n,d in sorted(members.items()):out.writestr(n,d)
        out.writestr('source_manifest.json',pretty(manifest))
    receipt=dict(schema='runtime_cost_r411_bundle_receipt_v1',files=len(members),bytes=(HERE/'bundle_sources.zip').stat().st_size,
        archive_sha256=sha((HERE/'bundle_sources.zip').read_bytes()),manifest_sha256=sha(pretty(manifest)),
        bootstrap_sha256=sha((HERE/'remote_bootstrap.py').read_bytes()),input_contract_sha256=input_contract,
        source_package23_matches_r406=True,static_ast_passed=True,control_calls=0,remote_executed=False)
    (HERE/'bundle_receipt.json').write_bytes(pretty(receipt));print(json.dumps(receipt))


if __name__=='__main__':main()
