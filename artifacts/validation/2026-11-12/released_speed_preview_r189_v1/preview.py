"""RELEASED의 원 정지를 유지하며 낮은 목표의 완전 관측 이동 영역만 비교."""
import hashlib
import importlib.util
import json
import math
from collections import Counter
from pathlib import Path

from arena_autonomy import pass_release_trial as ps
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output

A=Path(__file__).resolve().parent
TRACE=A.parent/'expiry_route_runtime_r187_v1/two_actor_run1/run1/follow_decisions.jsonl'


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def lower_observed(owner,scene,baseline,selected,speed,previous,dt):
    detail=selected[2];normal=baseline[2];partial=normal.get('partial_observation') or {}
    if not (selected[0]<=0 and owner.mode=='RELEASED'
            and owner.follow_config.traffic_recovery_trial
            and detail.get('selector_reason')=='large_motion_region_unobserved'
            and all(not side['present'] for side in owner.sides.values())
            and baseline[0]>0 and normal.get('reason') in ps.NORMAL_BASELINE
            and normal.get('boundary_from_memory') is False
            and normal.get('path_source_s')==scene.source_first_s
            and partial.get('allowed') is True and partial.get('reason')=='fully_observed'):
        return None
    if not all(isinstance(v,(float,int)) and not isinstance(v,bool) and math.isfinite(v)
               for v in (speed,previous,dt,baseline[0],baseline[1])) or not 0<=speed<=.35:
        return None
    attempts=[]
    for cap in (.35,.25,.15):
        if cap>=baseline[0] or cap<speed or cap>owner.config.guard_speed_cap_mps:continue
        region=ps.motion_region(scene,speed,cap,baseline[1],previous,dt,owner.follow_config,owner.lat)
        reason=ps.region_blocked(region,owner.config)
        attempts.append(dict(cap=cap,region=region,reason=reason,accepted=reason is None))
        if reason is None:break
    return dict(accepted=bool(attempts) and attempts[-1]['accepted'],attempts=attempts,
                speed=speed,original_target=baseline[0],steering=baseline[1],previous=previous,
                original_region=detail.get('selector_region'),source=scene.source_first_s,
                reference=scene.t_ref_s,unknown_filled=False)


def main():
    load('r189_parent',A.parent/'guard_approach_cap_r181_v1/candidate.py').install()
    route=load('r189_route',A.parent/'expiry_route_reference_r186_v1/candidate_v2.py');route.install()
    original=ps.PassReleaseSelector.select
    rows=[];current={}
    def shadow(self,scene,baseline,parallel,speed,previous,dt,*,raw_scan=None):
        selected=original(self,scene,baseline,parallel,speed,previous,dt,raw_scan=raw_scan)
        value=lower_observed(self,scene,baseline,selected,speed,previous,dt)
        if value is not None:rows.append(dict(seq=current['seq'],**value))
        return selected
    ps.PassReleaseSelector.select=shadow
    events=[json.loads(line) for line in TRACE.open()]
    owner,motion=make_controller(events[0]['data']);calls=0;mismatches=[]
    for event in events[1:]:
        d=event['data'];current['seq']=event['seq']
        if event['kind']=='reset':
            if d.get('reason')=='decision_source_expired':route.reset_reference(owner)
            else:owner.reset()
        if event['kind']!='call':continue
        result=freeze(replay_call(owner,d['input'],motion));calls+=1
        if not equal_output(comparable_decision(result),comparable_decision(d['output'])):mismatches.append(event['seq'])
    summary=dict(calls=calls,mismatches=mismatches,eligible=len(rows),
                 accepted=sum(row['accepted'] for row in rows),
                 reasons=dict(Counter(row['attempts'][-1]['reason'] if row['attempts'] else 'no_cap' for row in rows)))
    result=dict(**summary,rows=rows,trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),
                scope='shadow comparison only; original output and state retained; not a driving test')
    with (A/'result.json').open('x') as out:json.dump(result,out,indent=2);out.write('\n')
    print(json.dumps(summary));assert not mismatches


if __name__=='__main__':main()
