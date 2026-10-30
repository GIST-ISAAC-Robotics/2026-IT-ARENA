"""실행하지 않고 보존 입력의 G1 비교용 case 규칙 별칭 누락만 복구한다."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import types
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import numpy as np
from audit_local_pursuit_road import road_polygon
from merge_actor import TRIAL


def add_legacy_alias(config):
    fixed=copy.deepcopy(config)
    name=fixed['cases'][0]['name']
    if name!='g2_center_s_merge' or name in fixed['acceptance']:
        raise ValueError('unexpected case or existing acceptance')
    rules=fixed['acceptance']['g2_center_stopped']
    if rules!=fixed['acceptance']['stopped_center']:
        raise ValueError('legacy reference rules differ')
    fixed['acceptance'][name]=copy.deepcopy(rules)
    return fixed


def jsonl(path):
    with path.open() as stream:return [json.loads(line) for line in stream]


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    output=BASE/'run1'
    paths=[output/'report.json',BASE/'run1_g2_evaluation.json',BASE/'g2_center_config_v1.json',BASE/'run1_sources.zip']
    before={str(p.relative_to(BASE)):sha(p) for p in paths}
    original=json.loads(paths[0].read_text())
    assert original['evaluation']=={'error':"KeyError: 'minimum_continuous_hold_s'"}
    config=add_legacy_alias(json.loads(paths[2].read_text()))
    receipt=json.loads((output/'merge_actor_receipt.json').read_text())
    TRIAL.frame=tuple(receipt['frame']);TRIAL.scenario.start=receipt['scenario']['start_sim_s']
    TRIAL.observed_trigger=receipt['trigger']
    with zipfile.ZipFile(paths[3]) as archive:source=archive.read('effective_g1_traffic_eval.py').decode('utf-8')
    evaluator=types.ModuleType('merge_saved_g1_eval');evaluator.__file__=str(ROOT/'scripts/g1_traffic_eval.py')
    sys.modules[evaluator.__name__]=evaluator
    exec(compile(source,evaluator.__file__,'exec'),evaluator.__dict__)
    world=ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture'
    data=np.genfromtxt(world/'centerline.csv',delimiter=',',names=True)
    scene=json.loads((world/'scene.json').read_text())
    centerline=evaluator.Centerline(data['x_m'],data['y_m'],data['s_m'],scene['track']['lap_length_m'])
    green=original['green_observed_sim_s'];s0=original['route_check']['actor_start_s_m'];case=config['cases'][0]
    evaluated=evaluator.evaluate_run(config=config,case=case,mode='g1',t_green=green,
        trace=json.loads((output/'trace.json').read_text()),ego=jsonl(output/'ego_poses.jsonl'),
        actor=jsonl(output/'actor_poses.jsonl'),contacts=jsonl(output/'actor_contacts.jsonl'),
        commands=json.loads((output/'actor_commands.json').read_text()),stop_start=original['stop_request_sim_s'],
        schedule_pose=lambda t:TRIAL.pose(config,centerline,s0,case,t-green)[:2],centerline=centerline,s0=s0,
        road=road_polygon(output/'runtime_world.sdf'),runtime_world=(output/'runtime_world.sdf').read_text(),
        contact_evidence=original['contact_evidence'])
    report=copy.deepcopy(original);report['evaluation']=evaluated
    evaluator.finalize_verdict(report,'g1')
    result=json.loads(paths[1].read_text());prior=copy.deepcopy(result)
    result['gates']['harness_valid']=report['harness_valid']
    result['run_checks']['no_runtime_error']=report['integrity']['no_runtime_error']
    result['harness']=evaluated.get('harness')
    result['original_g1_after_alias']=dict(passed=report['passed'],gates=report['gates'],integrity=report['integrity'])
    result['evaluator']='g2_s_merge_saved_alias_v2'
    result['saved_reevaluation']=dict(scope='same drive records; only missing legacy G1 comparison rule alias',
        source_sha256=sha(Path(__file__)),original_sha256=before,
        original_files_unchanged=all(sha(BASE/p)==h for p,h in before.items()),
        alias=config['acceptance']['g2_center_s_merge'],
        prior_failure=original['evaluation']['error'],new_drive=False,
        changed_gates={k:[prior['gates'][k],v] for k,v in result['gates'].items() if prior['gates'][k]!=v},
        changed_run_checks={k:[prior['run_checks'][k],v] for k,v in result['run_checks'].items() if prior['run_checks'][k]!=v})
    result['passed']=bool(all(result['gates'].values()) and all(result['run_checks'].values())
        and result['saved_reevaluation']['original_files_unchanged'])
    with (BASE/'run1_evaluation_v2.json').open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(dict(passed=result['passed'],harness=result['harness'],
        changes=result['saved_reevaluation'],failed_gates=[k for k,v in result['gates'].items() if not v],
        failed_checks=[k for k,v in result['run_checks'].items() if not v])))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
