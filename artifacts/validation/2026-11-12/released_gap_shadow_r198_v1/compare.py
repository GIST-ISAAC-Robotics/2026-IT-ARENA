"""r187 RELEASED 정체에 기존 국소 후측방 공백 분류를 대조. 원 명령/상태 불변."""
import hashlib
import importlib.util
import json
import math
from collections import Counter
from pathlib import Path
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.current_path_recheck._gaps import bounded_body_nonreturn, bounded_adjacent_structure
from arena_autonomy.known_structure import parse_sectors
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output

A=Path(__file__).resolve().parent
TRACE=A.parent/'expiry_route_runtime_r187_v1/two_actor_run1/run1/follow_decisions.jsonl'

def compare(owner,scene,raw_scan,baseline,selected,speed,previous,dt):
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
               for v in (speed,previous,dt,baseline[0],baseline[1])) or not 0<=speed<=.35:return None
    attempts=[]
    for cap in (.35,.25,.15):
        if cap>=baseline[0] or cap<speed or cap>owner.config.guard_speed_cap_mps:continue
        context={}
        region=ps.motion_region(scene,speed,cap,baseline[1],previous,dt,owner.follow_config,owner.lat,context=context)
        proof=dict(accepted=False,reason='no_unknown_context')
        if region['unobserved'] and context:
            q=context['queries'][~context['free'] & ~context['masked']]
            proof=bounded_body_nonreturn(scene,raw_scan,q,context['padding'],
                owner.follow_config.length_m,owner.follow_config.width_m,speed,cap)
            if owner.follow_config.known_structure_policy=='structure_aware_v1':
                proof=bounded_adjacent_structure(scene,raw_scan,q,context['padding'],
                    owner.follow_config.length_m,owner.follow_config.width_m,speed,cap,
                    parse_sectors(owner.follow_config.known_structure_sectors_json))
        reason=ps.region_blocked(region,owner.config,bounded_unknown_risk=proof.get('accepted') is True)
        attempts.append(dict(cap=cap,region=region,proof=proof,reason=reason,accepted=reason is None))
        if reason is None:break
    return dict(accepted=bool(attempts) and attempts[-1]['accepted'],attempts=attempts,
        speed=speed,original_target=baseline[0],steering=baseline[1],previous=previous,
        source=scene.source_first_s,reference=scene.t_ref_s,unknown_filled=False)

def main():
    spec=importlib.util.spec_from_file_location('r198_route',A.parent/'expiry_route_reference_r186_v1/candidate_v2.py')
    route=importlib.util.module_from_spec(spec);spec.loader.exec_module(route);route.install()
    original=ps.PassReleaseSelector.select;rows=[];current={}
    def shadow(self,scene,baseline,parallel,speed,previous,dt,*,raw_scan=None):
        selected=original(self,scene,baseline,parallel,speed,previous,dt,raw_scan=raw_scan)
        value=compare(self,scene,raw_scan,baseline,selected,speed,previous,dt)
        if value is not None:rows.append(dict(seq=current['seq'],**value))
        return selected
    ps.PassReleaseSelector.select=shadow
    events=[json.loads(line) for line in TRACE.open()]
    events[0]['data']['follow_config']['guard_observation_policy']='selected_r181'
    owner,motion=make_controller(events[0]['data']);calls=0;mismatches=[]
    try:
        for event in events[1:]:
            data=event['data'];current['seq']=event['seq']
            if event['kind']=='reset':
                if data.get('reason')=='decision_source_expired':route.reset_reference(owner)
                else:owner.reset()
            if event['kind']!='call':continue
            actual=freeze(replay_call(owner,data['input'],motion));calls+=1
            if not equal_output(comparable_decision(actual),comparable_decision(data['output'])):
                mismatches.append(event['seq'])
    finally:ps.PassReleaseSelector.select=original
    result=dict(calls=calls,mismatches=mismatches,eligible=len(rows),accepted=sum(r['accepted'] for r in rows),
        final_reasons=dict(Counter(r['attempts'][-1]['proof']['reason'] if r['attempts'] else 'no_cap' for r in rows)),
        rows=rows,trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),
        scope='saved_input_shadow_only_original_output_and_state_unchanged_no_driving')
    with (A/'result.json').open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}));assert not mismatches

if __name__=='__main__':main()
