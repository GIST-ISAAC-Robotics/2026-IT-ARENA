"""격리 native 사본의409 simulation 동일성 + 제한 estimated 계약/반례. 장치 실행 없음."""
from pathlib import Path
import copy
import hashlib
import json
import math
import sys
import traceback

from arena_autonomy.runtime_dual_state import current_request
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.race_runtime_policy import crop_current_candidate as crop
from arena_autonomy.crop_edge_selection import ESTIMATED_SCOPE, component_scope

ROOT=Path(__file__).resolve().parent
def encoded(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def digest(value):return hashlib.sha256(encoded(value)).hexdigest()


def estimated_header(header):
    h=copy.deepcopy(header)
    h.update(observation_input_scope=ESTIMATED_SCOPE,runtime_time_policy='host_estimated_v1',
        current_corner_environment=dict(use_sim_time=False,motion_timing_verified=False,backend='native2slot_v1',
            execution_environment='hardware_runtime_candidate_v1',lidar_input_representation='native_estimated_v1',motion_mode='both'))
    return h


def estimated_record(record):
    """실물 입력이 아니라 동일 수치의 계약 사본이다. 원 파일을 수정하지 않는다."""
    r=copy.deepcopy(record)
    for snapshot in (r['plan_input'],r['current']):
        snapshot.update(timing_verified=False,timing_policy='host_estimated_v1')
        snapshot['scan']['measurements']['time_basis']='unverified_estimate'
    component=r['plan_output'][2].get('normal',{}).get('partial_observation',{}).get('crop_edge_component')
    if component is not None:component['scope']=component_scope(ESTIMATED_SCOPE)
    return r


def request(record):
    return current_request(record['plan_output'],record['plan_input'],record['current'],
        retained=record['retained'],crop_current_selection=True)


def normalized_plan(value):
    value=copy.deepcopy(value)
    component=value[2].get('normal',{}).get('partial_observation',{}).get('crop_edge_component')
    if component is not None:component['scope']='compared_scope_only'
    return value


def main():
    output=ROOT/'saved_result';output.mkdir(exist_ok=False)
    records=[json.loads(line) for name in ('r437','r426') for line in (ROOT/'inputs'/f'{name}_currents.jsonl').open()]
    expected={row['dataset']+':'+str(row['trace_seq']):row['output']
              for row in map(json.loads,(ROOT/'inputs/r440_rows.jsonl').open())}
    headers={name:json.loads((ROOT/'inputs'/f'{name}_header.json').read_bytes()) for name in ('r437','r426')}
    workers={};sim_count=0;cases=[];error=None
    try:
        with (output/'simulation.jsonl').open('x') as stream:
            for record in records:
                dataset=record['dataset'];key=dataset+':'+str(record['trace_seq'])
                if dataset not in workers:workers[dataset]=Evaluator('current',headers[dataset])
                data=request(record);before=digest(data);result=workers[dataset](data)
                assert result==expected[key],(key,'existing simulation output changed')
                assert digest(data)==before,(key,'input mutated')
                sim_count+=1;stream.write(json.dumps(dict(key=key,exact=True,input_sha256=before,output_sha256=digest(result)))+'\n')
        representative=next(r for r in records if r['dataset']=='r437' and r['trace_seq']==643)
        est=Evaluator('current',estimated_header(headers['r437']));workers['estimated']=est
        original_try=crop._try_once;calls=[]
        def observed(*args,**kwargs):
            calls.append(dict(cap=kwargs['target_cap'],measured=args[1]['current']['speed_mps']))
            return original_try(*args,**kwargs)
        crop._try_once=observed
        try:
            for fault in ('unchanged','front_obstacle','side_obstacle','expanded_unknown','epoch_mismatch',
                          'different_full_scan','verified_forgery'):
                record=estimated_record(representative);members=set(record['plan_output'][2]['normal']['partial_observation']
                    ['crop_edge_component']['source_indices'])
                if fault in ('front_obstacle','side_obstacle','expanded_unknown'):
                    for snapshot in (record['plan_input'],record['current']):
                        raw=snapshot['scan']['measurements']
                        for index,angle in enumerate(raw['angles_rad']):
                            degrees=math.degrees(angle)
                            choose=((fault=='front_obstacle' and abs(degrees)<10)
                                or (fault=='side_obstacle' and 80<degrees<125 and index not in members)
                                or (fault=='expanded_unknown' and index==max(members)+1))
                            if choose:raw['ranges_m'][index]={'nonfinite':'inf'} if fault=='expanded_unknown' else .12
                elif fault=='epoch_mismatch':record['current']['epoch']+=1
                elif fault=='different_full_scan':record['current']['scan']['measurements']['ranges_m'][0]=.12
                elif fault=='verified_forgery':record['current']['timing_verified']=True
                data=request(record);before=digest(data);calls.clear()
                if fault=='verified_forgery':
                    try:est(data)
                    except ValueError as exc:cases.append(dict(case=fault,rejected=str(exc)))
                    else:raise AssertionError('verified forgery accepted')
                    assert not calls
                    continue
                result=est(data);trial=result.get(crop.DIAGNOSTIC) or {}
                if fault=='unchanged':
                    assert result['speed_mps']==.1 and calls==[dict(cap=.1,measured=record['current']['speed_mps'])]
                    assert trial['attempted_targets']==1 and trial['final_full_checks']==1 and trial['proof']['accepted']
                    assert trial['timing_verified'] is False and trial['timing_policy']=='host_estimated_v1'
                else:assert result['speed_mps']==0,(fault,result)
                assert len(calls)<=1 and all(c['cap']==.1 for c in calls)
                assert digest(data)==before
                cases.append(dict(case=fault,calls=list(calls),input_sha256=before,output=result))
        finally:crop._try_once=original_try
        positive=next(r for r in records if expected[r['dataset']+':'+str(r['trace_seq'])]['speed_mps']>.1
            and crop.DIAGNOSTIC not in expected[r['dataset']+':'+str(r['trace_seq'])]
            and 'current_corner_distance_budget' not in expected[r['dataset']+':'+str(r['trace_seq'])])
        positive_worker=Evaluator('current',estimated_header(headers[positive['dataset']]))
        workers['positive']=positive_worker
        data=request(estimated_record(positive));before=digest(data);result=positive_worker(data)
        assert result==expected[positive['dataset']+':'+str(positive['trace_seq'])]
        assert digest(data)==before
        cases.append(dict(case='existing_positive_exact',dataset=positive['dataset'],trace_seq=positive['trace_seq'],output=result))
        sim_plan=Evaluator('plan',headers['r437']);workers['sim_plan']=sim_plan
        a=sim_plan(representative['plan_input']);sim_plan.close();del workers['sim_plan']
        est_plan=Evaluator('plan',estimated_header(headers['r437']));workers['est_plan']=est_plan
        converted=estimated_record(representative);b=est_plan(converted['plan_input'])
        assert normalized_plan(a)==normalized_plan(b),'plan numeric result differs beyond explicit scope'
        cases.append(dict(case='fresh_plan_contract_pair',simulation=a,estimated=b,
            same_numeric_result=True,physical_measurement=False))
        # 아래 수치는 가정 민감도이며 센서 오차 상한/교정값이 아니다. wheel/gyro 값을 합성하지 않는다.
        for label,offset,yaw in [('scan_minus_10ms',-.01,0.),('scan_plus_10ms',.01,0.),
                                 ('yaw_minus_half_degree',0.,-math.radians(.5)),
                                 ('yaw_plus_half_degree',0.,math.radians(.5))]:
            record=estimated_record(representative);h=estimated_header(headers['r437'])
            if offset:
                for snapshot in (record['plan_input'],record['current']):
                    raw=snapshot['scan']['measurements']
                    raw['acquired_at_s']=[t+offset for t in raw['acquired_at_s']]
            if yaw:h['motion_config']['lidar_yaw_rad']+=yaw
            worker=Evaluator('current',h);workers[label]=worker
            data=request(record);before=digest(data);result=worker(data);trial=result.get(crop.DIAGNOSTIC) or {}
            if trial.get('applied'):
                assert result['speed_mps']<=.1 and trial['attempted_targets']==1
            assert digest(data)==before and data['current']['timing_verified'] is False
            cases.append(dict(case=label,scan_offset_s=offset,lidar_yaw_offset_rad=yaw,
                physical_measurement=False,error_bound_verified=False,input_sha256=before,output=result))
    except BaseException as exc:
        error=repr(exc);(output/'traceback.txt').write_text(traceback.format_exc())
    finally:
        for worker in workers.values():worker.close()
    imports={name:str(Path(module.__file__).resolve()) for name,module in sys.modules.items()
        if name.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(module,'__file__',None)}
    external={name:path for name,path in imports.items() if not Path(path).is_relative_to(ROOT/'src')}
    report=dict(passed=error is None and sim_count==409 and len(cases)==13 and not external,
        error=error,simulation_exact=sim_count,estimated_cases=len(cases),external_imports=external,
        native_execution='native_v1',timing_accuracy_verified=False,ros_started=False,devices_opened=False)
    (output/'cases.json').write_text(json.dumps(cases,ensure_ascii=False,indent=2))
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
