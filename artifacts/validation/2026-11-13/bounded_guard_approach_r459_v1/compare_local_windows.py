"""실제 호출 당시 복제 owner/입력을 Windows Python에서 대조한다. native 전체 plan 검증과 별개."""
import copy
import hashlib
import json
from pathlib import Path
import pickle
import sys

B=Path(__file__).resolve().parent
sys.path[:0]=[str(B/'stage/src/arena_autonomy'),str(B/'stage/src/arena_vehicle_interface')]
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.follow_trace import freeze
from arena_autonomy.numeric_kernels import configure
configure('python')
source=B.parent/'guard_crawl_entry_r456_v1/pinpoint_v1/local_actual_snapshots.pkl'
snapshots=pickle.loads(source.read_bytes())
out=B/'local_windows_v1';out.mkdir(exist_ok=False)
checks=[]
for identity,snapshot in snapshots:
    owner,scene,baseline,selected,speed,previous,dt,raw_scan,measured=copy.deepcopy(snapshot)
    before=pickle.dumps(owner,protocol=5)
    detail={k:selected[2][k] for k in ('selector_mode_before','pass_release_trial','selector_unknown_filled',
        'selector_hidden_rear_risk_unresolved','selector_mode','selector_evidence','selector_sides') if k in selected[2]}
    parallel=(selected[0],selected[1],dict(selected[2],reason=selected[2]['selector_guard_candidate']['trigger']))
    view=ps.threat_view(scene,baseline[2]['path_points'],owner.lat)
    result=owner._guard_curve(scene,view,baseline,parallel,speed,previous,dt,scene.t_ref_s-scene.source_first_s,detail,raw_scan)
    assert before==pickle.dumps(owner,protocol=5)
    proof=result[2].get('selector_guard_candidate') or {}
    checks.append(dict(generation=identity['generation'],sequence=identity['sequence'],reference_s=identity['reference_s'],
        measured=measured,output=freeze(result),accepted=result[0]>0,cap=proof.get('target_speed_mps'),
        original_selected_speed=selected[0],owner_state_unchanged=True))
(out/'checks.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
import collections
summary=dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),rows=len(checks),
    accepted=sum(r['accepted'] for r in checks),caps=dict(collections.Counter(r['cap'] for r in checks)),
    reasons=dict(collections.Counter(r['output'][2].get('selector_reason') for r in checks)),
    whole_plan_replayed=False,backend='Windows Python; native replay pending WSL resource',new_drive=False)
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8');print(json.dumps(summary))
