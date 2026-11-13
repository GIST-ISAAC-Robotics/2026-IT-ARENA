"""완전 plan 제출 원문 재생과 실제 helper 진입 인자 관찰. 새 주행은 하지 않는다."""
import ast
import copy
import hashlib
import inspect
import json
import os
from pathlib import Path
import pickle
import sys

B=Path(__file__).resolve().parent
sys.path[:0]=[str(B/'base/src/arena_autonomy'),str(B/'base/src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import freeze
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.race_runtime_policy import guard_crawl as gc
from arena_autonomy import pass_release_trial as ps

O=B/'pinpoint_v1'
O.mkdir(exist_ok=False)
def write(n,v):(O/n).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
digest=lambda o: hashlib.sha256(pickle.dumps(o,protocol=5)).hexdigest()
source=inspect.getsource(gc.assess)
parsed=ast.parse(source).body[0]
conditions=[node.test for node in parsed.body if isinstance(node,ast.If)]
clauses=[(ast.unparse(node),compile(ast.Expression(node),'<actual guard_crawl condition>','eval')) for cond in conditions for node in cond.values]
original_assess=gc.assess
active=None
entries=[]
snapshots=[]

def observe(owner,scene,baseline,selected,speed,previous,dt,raw_scan,*,measured):
    result=original_assess(owner,scene,baseline,selected,speed,previous,dt,raw_scan,measured=measured)
    if active['input']['reference_s']<34.999:
        return result
    cfg=owner.follow_config
    normal,detail=baseline[2],selected[2]
    partial=normal.get('partial_observation') or {}
    original=detail.get('selector_guard_candidate') or {}
    retry=detail.get('selector_guard_speed_recheck') or {}
    attempts=retry.get('attempts') or []
    numbers=(measured,speed,previous,dt,baseline[0],baseline[1],selected[0],selected[1])
    regions=[original.get('region') or {}]+[a.get('region') or {} for a in attempts]
    scope=dict(locals())
    # Comprehensions use globals for free names; all values are actual call locals.
    scope={**gc.__dict__,**scope}
    failed=[text for text,code in clauses if eval(code,scope,scope)]
    item=dict(generation=active['generation'],sequence=active['sequence'],reference_s=active['input']['reference_s'],
        measured=measured,speed=speed,baseline_speed=baseline[0],baseline_reason=normal.get('reason'),
        baseline_behavior=normal.get('behavior'),baseline_boundary_from_memory=normal.get('boundary_from_memory'),
        baseline_path_source_s=normal.get('path_source_s'),scene_source_first_s=scene.source_first_s,
        selected_speed=selected[0],selected_reason=detail.get('reason'),selector_reason=detail.get('selector_reason'),
        previous_attempts=attempts,failed_actual_conditions=failed,original_helper_return_is_selected=result is selected)
    # Isolated extra .1 geometry only for original signed nonnegative scope.
    # No reconstruction of baseline/selector metadata from the logged normal.
    if 0<=measured<=.15:
        snapshot=copy.deepcopy((owner,scene,baseline,selected,speed,previous,dt,raw_scan,measured))
        snapshots.append((copy.deepcopy(item),snapshot))
        co,cs,cb,cp,cv,cprev,cdt,craw,cm=snapshot
        before=digest(co)
        selector_detail={k:cp[2][k] for k in ('selector_mode_before','pass_release_trial','selector_unknown_filled',
            'selector_hidden_rear_risk_unresolved','selector_mode','selector_evidence','selector_sides','selector_guard_speed_recheck') if k in cp[2]}
        view=ps.threat_view(cs,cb[2].get('path_points'),co.lat)
        parallel=(cp[0],cp[1],{**cp[2],'reason':original['trigger']})
        low=co._guard_curve_at_cap(cs,view,cb,parallel,cv,cprev,cdt,cs.t_ref_s-cs.source_first_s,selector_detail,craw,target_cap=.1)
        assert digest(co)==before
        item['independent_point_one']=freeze(low)
        item['candidate_copied_owner_unchanged']=True
    entries.append(item)
    return result

def diff(a,b,path='$'):
    if type(a) is dict and type(b) is dict:
        if a.keys()!=b.keys():return dict(path=path,actual_keys=list(a),expected_keys=list(b))
        for k in a:
            if a[k]!=b[k]:return diff(a[k],b[k],path+'.'+str(k))
    elif type(a) is list and type(b) is list:
        if len(a)!=len(b):return dict(path=path,actual_length=len(a),expected_length=len(b))
        for i,(x,y) in enumerate(zip(a,b)):
            if x!=y:return diff(x,y,path+f'[{i}]')
    return dict(path=path,actual=a,expected=b)

trace_path=B.parent/'guard_crawl_drive_r452_v1/run1/follow_decisions.jsonl'
rows=[json.loads(s) for s in trace_path.open()]
header=next(r['data']['header'] for r in rows if r['kind']=='runtime_evaluator_header')
done={(r['data']['generation'],r['data']['sequence']):r['data'] for r in rows if r['kind']=='runtime_plan_completed'}
submitted=[r for r in rows if r['kind']=='runtime_plan_submitted']
assert len(done)==len(submitted)==381
evaluator=None
generation=None
replayed=[]
mismatch=None
try:
    evaluator=Evaluator('plan',header)
    gc.assess=observe
    for row in submitted:
        active=row['data']
        if generation!=active['generation']:
            evaluator.reset(active['reset_reason'])
            generation=active['generation']
        before=digest(active['input'])
        output=evaluator(active['input'])
        expected=done[(generation,active['sequence'])]['output']
        assert digest(active['input'])==before
        exact=output==expected
        replayed.append(dict(trace_seq=row['seq'],generation=generation,sequence=active['sequence'],reference_s=active['input']['reference_s'],exact=exact))
        if not exact:
            mismatch=dict(identity=replayed[-1],difference=diff(output,expected))
            write('mismatch_actual.json',output);write('mismatch_expected.json',expected)
            break
finally:
    gc.assess=original_assess
    if evaluator is not None:evaluator.close()

import collections
write('replay_identity.json',replayed)
write('actual_entry_checks.json',entries)
(O/'local_actual_snapshots.pkl').write_bytes(pickle.dumps(snapshots,protocol=5))
summary=dict(pid=os.getpid(),source='r452 frozen archive; actual raw evaluator header and unique submitted input',
    replayed=len(replayed),exact=sum(e['exact'] for e in replayed),mismatch=mismatch,
    actual_stall_helper_calls=len(entries),failed_conditions=dict(collections.Counter(c for e in entries for c in e['failed_actual_conditions'])),
    measured_signs=dict(collections.Counter('negative' if e['measured']<0 else 'zero' if e['measured']==0 else 'positive' for e in entries)),
    local_point_one_trials=sum('independent_point_one' in e for e in entries),
    local_point_one_positive=sum(e.get('independent_point_one',[0])[0]>0 for e in entries),
    local_point_one_reasons=dict(collections.Counter(e['independent_point_one'][2].get('selector_reason') for e in entries if e.get('independent_point_one'))),
    module=gc.__file__,common_source_modified=False,policy_thresholds_changed=False,new_driving=False)
write('summary.json',summary)
print(json.dumps(summary,ensure_ascii=False),flush=True)
raise SystemExit(0 if mismatch is None and len(replayed)==381 else 2)
