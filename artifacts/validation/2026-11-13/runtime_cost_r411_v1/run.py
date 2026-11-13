"""r406 plan prefix37/current153의 현행 정책 OFF/ON 한 번. ROS/실시간 재생 아님."""
import argparse
from collections import Counter, defaultdict
import copy
import json
import platform
import signal
import sys
import time
import traceback
from diff_utils import comparison, recorded_result
from support import (HERE, ROOT, DIAGNOSTIC, current_invariants, digest, encoded,
                     import_receipt, statistics, verify_manifest, write_new)
from timed_evaluator import evaluator_factory


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--subset',choices=('smoke','full'),required=True);args=parser.parse_args()
    assert sys.platform=='linux' and platform.machine()==('x86_64' if args.subset=='smoke' else 'aarch64')
    out=HERE/args.subset;assert not out.exists(),'preserve first result'
    manifest=verify_manifest();out.mkdir()
    header=json.loads((HERE/'inputs/header.json').read_bytes())
    selection=json.loads((HERE/'inputs/selection.json').read_bytes())
    plans=[json.loads(s) for s in (HERE/'inputs/plans.jsonl').read_text(encoding='utf8').splitlines()]
    currents=[json.loads(s) for s in (HERE/'inputs/currents.jsonl').read_text(encoding='utf8').splitlines()]
    assert len(plans)==37 and len(currents)==153
    if args.subset=='smoke':currents=[currents[i] for i in (0,42,137)]
    from arena_autonomy.follow_trace import comparable_decision
    from arena_autonomy.runtime_dual_process import Lane
    began=time.monotonic();deadline=began+180.
    rows=[];comparisons=[];cleanups=[];imports={};error=None;completed={};source_errors=[];original_exact=True
    prior_handlers={}
    def interrupted(number,_):raise InterruptedError('signal_'+str(number))
    for number in (signal.SIGINT,signal.SIGTERM):prior_handlers[number]=signal.signal(number,interrupted)
    def compare(kind,index,expected,actual,stream,*,enforce=False):
        nonlocal original_exact
        value=comparison(expected,actual);record=dict(comparison_kind=kind,index=index,**value)
        stream.write(json.dumps(record,allow_nan=False)+'\n');stream.flush()
        comparisons.append({k:v for k,v in record.items() if k!='differences'})
        if kind.startswith('recorded_') and not value['exact_tree_equal']:original_exact=False
        if enforce and not value['exact_tree_equal']:raise AssertionError((kind,index,value['differences'][0]))
    try:
        with (out/'timing_rows.jsonl').open('x',encoding='utf8') as timing, (out/'comparison_rows.jsonl').open('x',encoding='utf8') as differences:
            for stage,records in (('plan',plans),('current',currents)):
                for arm in ('off','on'):
                    label=stage+'_'+arm
                    selected=dict(header,current_corner_budget='off' if arm=='off' else 'r396_v1')
                    if arm=='on':selected['current_corner_environment']=dict(use_sim_time=True,motion_timing_verified=True,backend='native2slot_v1')
                    else:selected.pop('current_corner_environment',None)
                    spawn_began=time.perf_counter_ns()
                    lane=Lane(stage,selected,capacity=4*1024*1024,timeout=20.,evaluator_factory=evaluator_factory)
                    startup_ms=(time.perf_counter_ns()-spawn_began)/1e6;completed[label]=[]
                    try:
                        for position,record in enumerate(records):
                            if time.monotonic()>=deadline:raise TimeoutError('bounded batch elapsed')
                            data=copy.deepcopy(record['data']);before_input=encoded(data)
                            offer_started=time.perf_counter_ns()
                            lane.offer(data,record['generation'],record['sequence'],record['reset_reason'])
                            offer_ms=(time.perf_counter_ns()-offer_started)/1e6
                            limit=min(deadline,time.monotonic()+25.)
                            while True:
                                lane.connection.poll(.01);received=lane.collect()
                                if received is not None:break
                                if time.monotonic()>=limit:raise TimeoutError((label,record['index'],'owned Lane wait'))
                            assert before_input==encoded(data),'parent input changed'
                            measured=received['output'];value=measured['output'];imports.update(measured['imports'])
                            assert measured['input_unchanged'] is True
                            row=dict(stage=stage,arm=arm,index=record['index'],trace_seq=record['trace_seq'],
                                generation=received['generation'],sequence=received['sequence'],input_sha256=record['data_sha256'],
                                evaluator_ms=measured['evaluator_ms'],
                                lane_with_audit_ms=1000*(received['completed_wall_s']-received['began_wall_s']),
                                delivery_with_audit_ms=1000*(received['received_wall_s']-received['submitted_wall_s']),
                                offer_encode_copy_ms=offer_ms,startup_ms=startup_ms if position==0 else None,
                                first_call=position==0,input_unchanged=True,selector_mode=record['selector_mode'],
                                output=value,state=measured['state'],eligible=measured['eligible'],source=record['source'])
                            # 비교 실패 전 원문부터 기록한다. 집계 오류가 계측 행을 잃게 하지 않는다.
                            timing.write(json.dumps(row,allow_nan=False)+'\n');timing.flush()
                            rows.append(row);completed[label].append(row)
                            if stage=='plan':
                                compare('recorded_plan_'+arm,record['index'],comparable_decision(record['expected']),
                                    comparable_decision(value),differences,enforce=args.subset=='smoke')
                                if arm=='on':
                                    old=completed['plan_off'][position]
                                    compare('same_machine_plan_output',record['index'],comparable_decision(old['output']),
                                        comparable_decision(value),differences,enforce=True)
                                    compare('same_machine_plan_state',record['index'],old['state'],measured['state'],differences,enforce=True)
                            elif arm=='on':
                                old=completed['current_off'][position]
                                outcome=current_invariants(old['output'],value,old['eligible']);row['outcome']=outcome
                                restored,cap_check=recorded_result(value,record['expected'])
                                if cap_check is not None:
                                    compare('recorded_parent_checked_target',record['index'],
                                        record['expected']['new_plan_deceleration_cap']['checked_target_mps'],
                                        value['speed_mps'],differences,enforce=args.subset=='smoke')
                                compare('recorded_current_on_after_parent_cap',record['index'],record['expected'],restored,
                                    differences,enforce=args.subset=='smoke')
                                # 원 cap은 비교용 후처리뿐이다. 계측 및 OFF/ON 결과는 raw evaluator다.
                                differences.write(json.dumps(dict(comparison_kind='same_machine_current_invariants',
                                    index=record['index'],passed=True,outcome=outcome,
                                    recorded_parent_cap=record['expected'].get('new_plan_deceleration_cap')),allow_nan=False)+'\n')
                                differences.flush()
                    finally:
                        cleanups.append(dict(label=label,startup_ms=startup_ms,**lane.close()))
                        assert not cleanups[-1]['forced'] and not cleanups[-1]['alive'] and cleanups[-1]['exitcode']==0
    except BaseException as exc:
        error=type(exc).__name__+': '+str(exc)
        (out/'traceback.txt').write_text(traceback.format_exc(),encoding='utf8')
    finally:
        for number,previous in prior_handlers.items():signal.signal(number,previous)
        try:verify_manifest();imports.update(import_receipt())
        except Exception as exc:source_errors.append(repr(exc))
        expected_count=80 if args.subset=='smoke' else 380
        computation_completed=len(rows)==expected_count
        invariant_passed=error is None and len(cleanups)==4
        groups=defaultdict(list)
        for row in rows:
            speed=row['output'][0] if row['stage']=='plan' else row['output']['speed_mps']
            prefix=row['stage']+'/'+row['arm']
            for key in (prefix,prefix+'/'+str(row['selector_mode']),prefix+('/positive' if speed>0 else '/zero')):
                groups[key].append(row['evaluator_ms'])
            if row['stage']=='current' and row['arm']=='on' and 'outcome' in row:groups[prefix+'/'+row['outcome']].append(row['evaluator_ms'])
        off={r['index']:r for r in completed.get('current_off',[])}
        paired=[dict(index=r['index'],off_ms=off[r['index']]['evaluator_ms'],on_ms=r['evaluator_ms'],
            delta_ms=r['evaluator_ms']-off[r['index']]['evaluator_ms'],outcome=r.get('outcome'))
            for r in completed.get('current_on',[]) if r['index'] in off]
        distributions={}
        for label,records in completed.items():
            stage=label.split('_')[0]
            distributions[label]=dict(n=len(records),modes=dict(Counter(r['selector_mode'] for r in records)),
                positive=sum((r['output'][0] if stage=='plan' else r['output']['speed_mps'])>0 for r in records),
                outcomes=dict(Counter(r.get('outcome','not_compared') for r in records)) if stage=='current' else {},
                reasons=dict(Counter(r['output']['reason'] for r in records)) if stage=='current' else {})
        applied=[r['index'] for r in completed.get('current_on',[]) if r['output'].get(DIAGNOSTIC,{}).get('applied')]
        categories=Counter()
        for row in comparisons:categories.update(row['categories'])
        report=dict(schema='runtime_cost_r411_result_v1',subset=args.subset,machine=platform.machine(),
            passed=computation_completed and invariant_passed and original_exact and not source_errors,
            cost_collection_completed=computation_completed and invariant_passed and not source_errors,
            computation_completed=computation_completed,same_machine_invariants_passed=invariant_passed,
            historical_exact_reproduction_passed=original_exact,error=error,source_errors=source_errors,
            completed_calls=len(rows),expected_calls=expected_count,selected_plan=37,
            selected_current_indices=[r['index'] for r in currents],applied_indices=applied,
            recorded_applied_indices=[137],applied_count_matches_recorded=applied==[137],
            distributions=distributions,evaluator_ms={k:statistics(v) for k,v in groups.items()},
            current_paired_differences=paired,first_calls=[{k:r[k] for k in ('stage','arm','index','evaluator_ms','startup_ms')} for r in rows if r['first_call']],
            diff_categories=dict(categories),detailed_comparisons=len(comparisons),cleanup=cleanups,imports=imports,
            manifest_sha256=digest(ROOT/'source_manifest.json'),input_contract_sha256=manifest['input_contract_sha256'],
            selection=selection,elapsed_s=time.monotonic()-began,automatic_adoption_allowed=False,
            original_r406_passed=False,original_r382_pipeline_passed=False,command_expiry_evaluated=False,
            physical_devices_opened=False,real_sensor_inputs=False,
            scope='single serial evaluator cost; Lane includes audit wrapper, not production IPC or real-time latency')
        write_new(out/'report.json',report)
        print(json.dumps({k:report[k] for k in ('subset','passed','cost_collection_completed','historical_exact_reproduction_passed','completed_calls','applied_indices','error')}))
    if error or source_errors or not computation_completed or (args.subset=='smoke' and not report['passed']):raise SystemExit(1)


if __name__=='__main__':main()
