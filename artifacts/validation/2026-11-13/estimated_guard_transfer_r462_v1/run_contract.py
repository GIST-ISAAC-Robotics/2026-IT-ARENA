"""승인된 격리 ARM 사본에서 소수 저장 계약만 비교한다. ROS/장치 사용 없음."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import pickle
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from contract_tools import load_converters, assert_snapshot_numbers_unchanged
from diff_utils import comparison


def read(path):
    return json.loads(path.read_bytes())


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def write(path, value):
    with path.open('x',encoding='utf8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2)
        stream.write('\n')


def run(contract):
    from arena_autonomy.race_runtime_policy.evaluator import Evaluator
    from arena_autonomy.runtime_dual_state import current_request
    from arena_autonomy.numeric_kernels import configure
    from arena_autonomy import pass_release_trial as ps
    from arena_autonomy.follow_trace import freeze, restore_call
    from arena_autonomy.lidar_motion import MotionConfig
    from arena_autonomy.scene_motion import motion_scene
    from arena_autonomy.crop_edge_selection import timing_matches
    output = ROOT/'results'/contract
    output.mkdir(parents=True,exist_ok=False)
    converters = load_converters(ROOT/'contract_reference.py')
    records = [json.loads(line) for line in (ROOT/'inputs/currents.jsonl').open()]
    workers, rows = {}, []
    try:
        for record in records:
            dataset = record['dataset']
            header = read(ROOT/'inputs'/f'{dataset}_header.json')
            selected = converters['estimated_record'](record) if contract=='estimated' else copy.deepcopy(record)
            if dataset not in workers:
                chosen = converters['estimated_header'](header) if contract=='estimated' else header
                workers[dataset] = Evaluator('current',chosen)
            if contract=='estimated':
                for key in ('plan_input','current'):
                    assert_snapshot_numbers_unchanged(record[key],selected[key])
            request = current_request(selected['plan_output'],selected['plan_input'],selected['current'],
                retained=selected['retained'],crop_current_selection=True)
            before = digest(request)
            begin = time.perf_counter()
            result = workers[dataset](request)
            elapsed = time.perf_counter()-begin
            assert digest(request)==before
            assert result['runtime_authorized'] is False and result['independent_guard_required'] is True
            rows.append(dict(id=record['id'],kind='actual_recorded_current',contract=contract,
                source_identity=record['identity'],input_sha256=before,input_unchanged=True,
                original_measured_mps=record['current']['speed_mps'],
                timing_verified=selected['current']['timing_verified'],
                timing_policy=selected['current'].get('timing_policy','verified_only'),
                time_basis=selected['current']['scan']['measurements']['time_basis'],
                selected_scope=workers[dataset].observation_scope,same_scan=request['crop_current_input']['same_scan'],
                evaluator_seconds=elapsed,output=result,
                x86_saved_comparison=comparison(record['expected'],result) if contract=='simulation' else None))
        if contract=='estimated':
            # 같은 저장 수치에서 timing flag만 위장한 입력은 기존 evaluator 입구가 거절해야 한다.
            source = records[0]
            forged = converters['estimated_record'](source)
            forged['current']['timing_verified'] = True
            request = current_request(forged['plan_output'],forged['plan_input'],forged['current'],
                retained=forged['retained'],crop_current_selection=True)
            try:
                workers[source['dataset']](request)
            except ValueError as error:
                write(output/'verified_forgery.json',dict(rejected=True,error=str(error),only_flag_changed=True))
            else:
                raise AssertionError('estimated true-flag forgery accepted')
    finally:
        for worker in workers.values():
            worker.close()
    write(output/'currents.json',rows)

    # 실제 owner checkpoint의 국소 _guard_curve 동등성. 전체 plan/새 current 재생이 아니다.
    configure('native_v1')
    class CompatibleUnpickler(pickle.Unpickler):
        def find_class(self,module,name):
            if module.startswith('numpy._core'):
                module=module.replace('numpy._core','numpy.core',1)
            return super().find_class(module,name)
    with (ROOT/'inputs/actual_owner_snapshots.pkl').open('rb') as stream:
        snapshots = CompatibleUnpickler(stream).load()
    identity,snapshot = next(x for x in snapshots if x[0]['generation']==360 and x[0]['sequence']==573)
    owner,old_scene,baseline,selected,speed,previous,dt,old_scan,measured = copy.deepcopy(snapshot)
    source = read(ROOT/'inputs/plan_checkpoint_input.json')
    chosen = converters['estimated_snapshot'](source) if contract=='estimated' else source
    if contract=='estimated':
        assert_snapshot_numbers_unchanged(source,chosen)
        assert timing_matches('host_estimated_crop_v1',chosen['timing_verified'],chosen['timing_policy'],
            chosen['scan']['measurements']['time_basis'])
    header = read(ROOT/'inputs/r452_header.json')
    info,scan,history = restore_call(chosen,MotionConfig(**header['motion_config']))
    scene = motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],
        epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'],
        timing_policy=info.get('timing_policy','verified_only'))
    before_owner = hashlib.sha256(pickle.dumps(owner,protocol=4)).hexdigest()
    detail = {k:selected[2][k] for k in ('selector_mode_before','pass_release_trial','selector_unknown_filled',
        'selector_hidden_rear_risk_unresolved','selector_mode','selector_evidence','selector_sides') if k in selected[2]}
    parallel = (selected[0],selected[1],dict(selected[2],reason=selected[2]['selector_guard_candidate']['trigger']))
    view = ps.threat_view(scene,baseline[2]['path_points'],owner.lat)
    result = owner._guard_curve(scene,view,baseline,parallel,speed,previous,dt,
        scene.t_ref_s-scene.source_first_s,detail,scan)
    assert hashlib.sha256(pickle.dumps(owner,protocol=4)).hexdigest()==before_owner
    write(output/'local_checkpoint.json',dict(identity={k:identity[k] for k in ('generation','sequence','reference_s')},
        contract=contract,output=freeze(result),measured_mps=measured,owner_state_unchanged=True,
        timing_verified=chosen['timing_verified'],timing_policy=chosen.get('timing_policy','verified_only'),
        time_basis=chosen['scan']['measurements']['time_basis'],whole_plan_replayed=False,current_synthesized=False))
    imported = {n:str(Path(m.__file__).resolve()) for n,m in sys.modules.items()
        if n.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(m,'__file__',None)}
    external = {n:p for n,p in imported.items() if not Path(p).is_relative_to(ROOT/'src')}
    assert not external,external
    write(output/'receipt.json',dict(completed=True,current_calls=len(rows),checkpoint_calls=1,
        invalid_contract_calls=int(contract=='estimated'),machine=os.uname().machine,physical_measurement=False,
        sensors_opened=False,ros_started=False,imports=imported,external_imports=external))


def summarize():
    values = {c:read(ROOT/'results'/c/'currents.json') for c in ('simulation','estimated')}
    comparisons=[]
    for a,b in zip(values['simulation'],values['estimated']):
        assert a['id']==b['id']
        fields=('speed_mps','steering_rad','reason','path_points','independent_guard_required','runtime_authorized')
        first={k:a['output'].get(k) for k in fields}
        second={k:b['output'].get(k) for k in fields}
        comparisons.append(dict(id=a['id'],decision_exact=first==second,
            decision_comparison=comparison(first,second),all_output_comparison=comparison(a['output'],b['output']),
            original_x86_comparison=a['x86_saved_comparison']))
    local = {c:read(ROOT/'results'/c/'local_checkpoint.json') for c in values}
    result = dict(current_cases=len(comparisons),same_arm_decision_exact=all(r['decision_exact'] for r in comparisons),
        local_checkpoint_comparison=comparison(local['simulation']['output'],local['estimated']['output']),
        local_checkpoint_numeric_exact=local['simulation']['output']==local['estimated']['output'],
        local_checkpoint_positive={c:local[c]['output'][0]>.0 for c in local},
        current_speeds={c:{r['id']:r['output']['speed_mps'] for r in values[c]} for c in values},
        current_reasons={c:{r['id']:r['output']['reason'] for r in values[c]} for c in values},
        verified_forgery_rejected=read(ROOT/'results/estimated/verified_forgery.json')['rejected'],
        current_comparisons=comparisons,physical_measurement=False,hardware_activation=False,
        timing_accuracy_verified=False,whole_plan_replay=False,
        note='GUARD existing .15 stage differs from estimated crop/corner fallback .1 target; no general speed cap change')
    write(ROOT/'results/summary.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('current_comparisons','local_checkpoint_comparison')}))


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('mode',choices=('simulation','estimated','summarize'))
    mode=parser.parse_args().mode
    summarize() if mode=='summarize' else run(mode)
