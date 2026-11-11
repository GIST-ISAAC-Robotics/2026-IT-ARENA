import hashlib
import importlib.util
import json
from pathlib import Path

from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, comparable_decision, freeze, equal_output, restore_call
from arena_autonomy.scene_motion import motion_scene
import arena_autonomy.rear_fringe_trial as rear
import fast_nonreturn as fast

A=Path(__file__).resolve().parent;ROOT=A.parents[3]


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def main():
    planner=load('r145_planner',ROOT/'artifacts/validation/2026-11-11/guard_recheck_r139_v1/candidate.py')
    planner.install()
    rear._bounded_nonreturn=fast._bounded_nonreturn
    planner.body._bounded_nonreturn=fast._bounded_nonreturn
    path=ROOT/'artifacts/validation/2026-11-11/guard_recheck_r139_v1/two_actor_run1/run1/follow_decisions.jsonl'
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    rows=[json.loads(line) for line in path.open()]
    owner,motion=make_controller(rows[0]['data']);calls=0;mismatch=[]
    for row in rows[1:-1]:
        if row['kind']=='reset':owner.reset()
        elif row['kind']=='call':
            calls+=1;actual=freeze(replay_call(owner,row['data']['input'],motion))
            if not equal_output(comparable_decision(actual),comparable_decision(row['data']['output'])):mismatch.append(row['seq'])
    guard=load('r145_guard',A.parent/'guard_current_r142_v1/candidate.py')
    guard.body._bounded_nonreturn=fast._bounded_nonreturn
    samples=[json.loads(line) for line in (A.parent/'guard_current_cost_r144_v1/samples.jsonl').open()]
    fresh_owner,motion=make_controller(samples[0]['header']);current_mismatch=[]
    for row in samples[1:]:
        data,scan,history=restore_call(row['current'],motion)
        scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                           epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
        result=freeze(guard.rebase(row['original']['output'],row['original']['input'],data,scan,scene,history,fresh_owner.config))
        if not equal_output(result,row['expected']):current_mismatch.append(row['seq'])
    result=dict(original_calls=calls,original_mismatches=mismatch,current_calls=len(samples)-1,
                current_mismatches=current_mismatch,trace_sha256=digest,
                source_unchanged=hashlib.sha256(path.read_bytes()).hexdigest()==digest,
                scope='Saved outputs and diagnostic equality; no source adoption or new driving proof')
    with (A/'verification.json').open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result));assert not mismatch and not current_mismatch and result['source_unchanged']


if __name__=='__main__':main()
