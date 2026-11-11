"""기존 소실 기억 검사를 통과한 경우만 후보 기하를 붙이는 저장재생용 실험."""
import json
import math
from pathlib import Path
import sys

import numpy as np

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.scan_input import scan_angles,scan_cell_widths,scan_max_gap
from probe import envelope


def clustered_front(scan,yaw,explained=None):
    angles=(scan_angles(scan)+yaw+math.pi)%math.tau-math.pi
    front=np.argsort(angles);front=front[abs(angles[front])<=math.radians(100)+1e-6]
    values=np.asarray(scan.ranges)
    missing=~(np.isfinite(values)&(values>=scan.range_min)&(values<=scan.range_max))
    if explained:missing[list(explained)]=False
    ids=np.flatnonzero(missing[front])
    if not len(ids):return False
    runs=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
    if len(runs)<2:return False
    cells=scan_cell_widths(scan);step=scan_max_gap(scan)
    groups=[run for run in runs if np.any(abs(angles[front[run]])<=math.radians(30)+step/2)]
    return sum(float(sum(cells[front[run]])) for run in groups)>math.radians(1.5)+1e-7


class PolicyHooks:
    def __enter__(self):
        self.evaluate=PartialObservationPolicy.evaluate
        self.request=PartialObservationPolicy.requested_speed
        original=self.evaluate;request=self.request
        def requested(owner,scan,yaw,target,explained=None):
            target=request(owner,scan,yaw,target,explained)
            if owner.lateral_metric_enabled and target>.2 and clustered_front(scan,yaw,explained):
                return .2
            return target
        def evaluate(owner,scan,points,history,now,speed,steering=0.,previous_steering=0.,
                     wheelbase=.135,length=.20,width=.15):
            result=original(owner,scan,points,history,now,speed,steering,previous_steering,wheelbase,length,width)
            if result.get('reason')!='forward_gap_too_wide' or owner.latched or not owner.lateral_metric_enabled:
                return result
            geometry=envelope(scan,history,now,speed,steering,previous_steering,wheelbase,length,width)
            if not geometry['eligible']:return {**result,'cluster_probe':geometry}
            # 원 함수의 현재점/근거리 소실/시간 이력/래치를 통과한 뒤만 후보로 삼는다.
            return {**result,'allowed':True,'reason':'bounded_cluster_forward_probe','speed_cap_mps':.2,
                'cluster_probe':geometry,'unknown_filled':False,'residual_hidden_object_risk':True}
        PartialObservationPolicy.evaluate=evaluate
        PartialObservationPolicy.requested_speed=requested
        return self
    def __exit__(self,*args):
        PartialObservationPolicy.evaluate=self.evaluate
        PartialObservationPolicy.requested_speed=self.request


def main():
    from arena_autonomy.decision_process import make_controller
    from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
    import hashlib
    source=A.parent/'forward_gap_scope_r109_v1/two_actor_run1/run1/follow_decisions.jsonl'
    digest=hashlib.sha256(source.read_bytes()).hexdigest()
    rows=[json.loads(s) for s in source.open(encoding='utf8')]
    calls=[r for r in rows if r['kind']=='call'];results={}
    for candidate in (False,True):
        controller,motion=make_controller(rows[0]['data']);out=[]
        from contextlib import nullcontext
        with PolicyHooks() if candidate else nullcontext():
            for row in rows[1:]:
                if row['kind']=='reset':controller.reset()
                elif row['kind']=='call':out.append(freeze(replay_call(controller,row['data']['input'],motion)))
        results[candidate]=out
    baseline=sum(not equal_output(comparable_decision(a),comparable_decision(r['data']['output'])) for a,r in zip(results[False],calls))
    deltas=[];candidates=[]
    for row,before,after in zip(calls,results[False],results[True]):
        item=dict(seq=row['seq'],reference_s=row['data']['input']['reference_s'],before=before[:2],after=after[:2])
        if not equal_output(before[:2],after[:2]):deltas.append(item)
        detail=after[2].get('normal',{});proof=detail.get('partial_observation',{})
        if proof.get('reason')=='bounded_cluster_forward_probe':
            candidates.append({**item,'normal_reason':detail.get('reason'),'selector_reason':detail.get('selector_reason'),
                'same_path_gap':detail.get('same_path_gap'),'proof':proof})
    assert hashlib.sha256(source.read_bytes()).hexdigest()==digest
    report=dict(source_sha256=digest,calls=len(calls),baseline_mismatches=baseline,
        command_changes=deltas,policy_candidates=candidates,
        scope='offline policy only, same-path consumer unchanged; no independent safety or closed-loop test')
    with (A/'policy_only.json').open('x',encoding='utf8') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(dict(calls=len(calls),baseline_mismatches=baseline,command_changes=len(deltas),policy_candidates=len(candidates))))


if __name__=='__main__':main()
