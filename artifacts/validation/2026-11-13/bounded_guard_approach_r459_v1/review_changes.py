"""native 원/수정 출력 차이의 원인과 명시 위험 예산을 읽기 전용으로 점검한다."""
import hashlib
import json
from pathlib import Path
B=Path(__file__).resolve().parent
plans=json.loads((B/'native_v1/plans.json').read_bytes())
review=[]
for row in plans:
    if row['exact']:continue
    normal=row['output'][2]['normal'];guard=normal.get('selector_guard_candidate') or {}
    approach=row['current_approach'];region=guard.get('region') or {}
    command_changed=not row['command_same']
    checks=dict(path_same=row['path_same'],side_state_same=row['side_state_same'])
    if command_changed:
        checks.update(old_zero=row['old_command'][0]==0,new_existing_cap=row['new_command'][0] in (.15,.35),
            guard_accepted=guard.get('accepted') is True,
            bounded_basis=approach.get('basis')=='observed_with_mask_budget',
            region_unknown_zero=region.get('unobserved')==0,front_mask_zero=region.get('masked_front_samples')==0,
            existing_tail_budget=region.get('masked_tail_extent_m',1)<=.01,
            measured_motion_preserved=guard.get('prediction_speed_mps')==max(abs(row['measured']),row['new_command'][0]),
            sides_have_free=all(s['currently_observed']>=1 for s in approach['sides'].values()),
            all_known_masks_bounded=all(s['known_masked']==s['bounded_mask'] and s['unaccepted_masked']==s['unknown']==0 for s in approach['sides'].values()),
            existing_hidden_width_span=all(s.get('masked_x_span_m',0)<=.05 for s in approach['sides'].values()))
    review.append(dict(generation=row['generation'],sequence=row['sequence'],reference_s=row['reference_s'],
        measured=row['measured'],old_command=row['old_command'],new_command=row['new_command'],
        classification='bounded_self_mask_permission' if command_changed else 'diagnostic_only_observed_counts',
        prediction_speed_mps=guard.get('prediction_speed_mps'),region=region,approach=approach,checks=checks,
        passed=all(checks.values())))
assert len(review)==112 and all(x['passed'] for x in review)
(B/'native_v1/change_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
R=B.parents[3];changes=json.loads((B/'changes.json').read_bytes())
common_unchanged=all((not (R/r['path']).exists()) if r['original_sha256'] is None else hashlib.sha256((R/r['path']).read_bytes()).hexdigest()==r['original_sha256'] for r in changes)
manifest=json.loads((B/'stage_manifest.json').read_bytes())
stage_unchanged=all(hashlib.sha256((B/'stage'/p).read_bytes()).hexdigest()==h for p,h in manifest.items())
summary=dict(reviewed_changes=len(review),command_changes=sum(r['classification']=='bounded_self_mask_permission' for r in review),
    diagnostic_only=sum(r['classification']=='diagnostic_only_observed_counts' for r in review),all_checks_passed=True,
    common_unchanged=common_unchanged,stage_unchanged=stage_unchanged,
    native_exit=int((B/'native_v1.exit').read_text()),post_exit_resource_check_pending=not (B/'resource_return.json').exists())
(B/'native_v1/change_review_summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8');print(json.dumps(summary))
assert common_unchanged and stage_unchanged
