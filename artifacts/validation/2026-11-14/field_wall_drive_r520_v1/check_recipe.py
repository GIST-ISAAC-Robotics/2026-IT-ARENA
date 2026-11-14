"""r520 Windows 정적 준비와 금지 상태를 확인한다. 최종 소스 선택/동결/WSL 실행과 구분한다."""
import ast
import argparse
import hashlib
import json
from pathlib import Path
import re

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
OLD=A.parent/'current_guide_extent_drive_r515_v1'
read=lambda p:json.loads(p.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main(final=False):
    cfg=read(A/'config.json');before=read(OLD/'config.json')
    diff=sorted(k for k in set(cfg)|set(before) if cfg.get(k)!=before.get(k))
    assert diff==['launch','registered_at','scope'] and cfg==read(A/'expected_config.json')
    launch=dict(cfg['launch']);launch['entry_g1']=before['launch']['entry_g1'];assert launch==before['launch']
    assert read(A/'arguments.json')==[v.replace(OLD.name,A.name) for v in read(OLD/'arguments.json')]
    selection=read(A/'source_selection.json')
    assert all(selection[k] is final for k in ('freeze_authorized','wsl_exclusive_owner','drive_authorized'))
    assert selection['final_production_source_selection_pending'] is (not final)
    provenance=read(A/'recipe_provenance.json')
    for item in provenance['files']:
        assert sha(A/item['path'])==item['after_sha256'],item['path']
        if item['byte_exact']:assert sha(A/item['path'])==sha(OLD/item['path'])
    for p,digest in provenance['original_evidence'].items():assert sha(ROOT/p)==digest,p
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    for p in [*A.rglob('*.sh'),*A.rglob('*.bash')]:assert b'\r' not in p.read_bytes(),p
    for name in ('PLAN.md','READY.md'):
        if (A/name).exists():
            missing=[p for p in re.findall(r'\]\(([^)#]+)',(A/name).read_text(encoding='utf-8')) if not (A/p).exists()]
            assert not missing,missing
    from field_wall_geometry import verify_derived
    field=verify_derived()
    assert read(A/'geometry_right_side.json')['passed'] and read(A/'field_static_geometry.json')['passed']
    run=(A/'run.py').read_text(encoding='utf-8')
    launch_source=(A/'launch/simulation.launch.py').read_text(encoding='utf-8')
    assert 'return original_obstacles(DERIVED)' in run
    assert "bound.arguments['source']=DERIVED" in launch_source
    assert launch_source.index("bound.arguments['source']=DERIVED")<launch_source.index('report=original(*bound.args,**bound.kwargs)')
    assert "assert collision_digest(destination)==info['walls_collision_sha256']" in launch_source
    assert not any('world_override:=' in x for x in read(A/'arguments.json'))
    baseline=read(OLD/'prepared.json')['frozen_inputs']
    common={p:h for p,h in baseline.items() if p.startswith(('src/','scripts/','tests/','config/')) or 'old_signal_610_r326_v1/' in p}
    delta={p:dict(before_sha256=h,after_sha256=sha(ROOT/p)) for p,h in common.items() if sha(ROOT/p)!=h}
    authorized={item['path']:item for item in selection['control_changes']}
    for p,item in authorized.items():
        assert delta[p]['before_sha256']==item['before_sha256'] and delta[p]['after_sha256']==item['after_sha256'],p
    for p,digest in selection.get('reviewed_noncontrol_hashes',{}).items():assert sha(ROOT/p)==digest,p
    for p,digest in selection.get('additional_test_hashes',{}).items():
        assert p not in baseline and sha(ROOT/p)==digest,p
    for p,digest in selection.get('additional_source_hashes',{}).items():
        assert p not in baseline and sha(ROOT/p)==digest,p
    unselected=sorted(set(delta)-set(authorized)-set(selection['noncontrol_changes']))
    if final:assert not unselected,unselected
    assert not any((A/n).exists() for n in ('prepared.json','sources.zip','registered.json','root_execution_review.json','run1','preflight'))
    result=dict(passed=True,scope='Windows generated-geometry/static preparation only; final controller selection and real WSL import not accepted here',
        config_changed_keys_vs_r515=diff,actor_facilities_physics_speed_stop_acceptance_unchanged=True,
        generated_world_sha256=field['derived_sha256'],static_actor_samples=601,static_initial_ego_passed=True,
        unchanged_original_walls=127,wall_collisions=field['derived_wall_collision_count'],
        evaluator_and_runtime_use_same_derived_path=True,injection_before_visual_batch=True,
        original_evidence_checked=len(provenance['original_evidence']),common_compared=len(common),
        observed_common_delta_not_final_selection=delta,root_selected_control_paths=sorted(authorized),
        unselected_common_delta=unselected,
        final_production_source_selection_pending=not final,live_runtime_import_tested=False,
        WSL_used=False,frozen=False,simulator_started=False,actual_run_count=0)
    (A/('final_preparation_receipt.json' if final else 'preparation_receipt.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('passed','wall_collisions','static_actor_samples','observed_common_delta_not_final_selection','unselected_common_delta','frozen','actual_run_count')}))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--final',action='store_true');main(parser.parse_args().final)
