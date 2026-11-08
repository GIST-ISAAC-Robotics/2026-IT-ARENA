"""r36 재출발의 첫 경로 선택: 같은 관측/여유에서 이미 등록된 전이만 비교."""
from pathlib import Path
import json
import numpy as np
from arena_autonomy.follow_trace import restore_call,motion_scene,MotionConfig,FollowConfig,freeze
from arena_autonomy.bypass_entry_trial import EntryConfig,_compare_entries

A=Path(__file__).resolve().parent
d=json.loads((A/'initial_snapshots.json').read_bytes());cap=d['captures'][0]
cfg=FollowConfig(**d['header']['follow_config'])
i,scan,history=restore_call(cap['input'],MotionConfig(**d['header']['motion_config']))
scene=motion_scene(scan,history,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
old=cap['output'][2]['bypass'];evidence=old['reference']
padding=np.asarray(cap['sweeps'][0]['padding'])
assert all(np.array_equal(padding,np.asarray(s['padding'])) for s in cap['sweeps'][:3])
assert len(cap['sweeps'])==6  # 뒤의 세 호출은 별도의 0.1초 성장 비교다.
assert np.allclose(np.asarray(cap['sweeps'][0]['points']),scene.occupied_endpoints_xy_m)
def preserved_padding(scene,config,side,wall,growth):
    assert side=='right' and growth==0
    return padding.copy(),dict(scope='exact_recorded_first_selection_padding')
result=_compare_entries(scene,scene.t_ref_s,i['speed_mps'],i['previous_steering_rad'],i['dt_s'],config=cfg,
    entry=EntryConfig(offsets_m=(0.,-.27),transitions_m=(.6,.7,.8,.9)),stationary_boundaries=True,
    padding_provider=preserved_padding,reference_provider=lambda scene,config:(True,evidence))
rows=[{k:v for k,v in row.items() if k not in ('path_points','outer_boundary_padding')}
      for row in result['candidates']]
out=dict(scope='saved first selection only; no changed-command continuation or driving claim',
    reference_s=scene.t_ref_s,maximum_steering_rad=cfg.max_steering_rad,
    original_selected=old['selected'],candidates=rows,comparison_selected=result['selected'])
(A/'initial_paths.json').write_text(json.dumps(freeze(out),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
for r in rows:
    print({k:r.get(k) for k in ('offset_m','transition_m','eligible','reason','preview_maximum_steering_rad','preview_return_progress_m','preview_point_gap_m','steering_rad','target_speed_mps')})

from short_transition_candidate import compare_short_transition
trial=compare_short_transition(scene,cfg,EntryConfig(offsets_m=(0.,-.27),transitions_m=(.6,.8,.9,1.2)),old["corridor_plan"]["rows"]["right"],evidence["walls"]["right"])
(A/"short_transition_result.json").write_text(json.dumps(trial,indent=2)+"\n",encoding="utf8")
print(trial)
