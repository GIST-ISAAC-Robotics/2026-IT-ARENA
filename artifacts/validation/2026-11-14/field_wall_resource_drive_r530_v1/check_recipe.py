"""r530 자원 경로/기하/실행 조건의 Windows 준비 검사. 제어 실행/WSL 사용 없음."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import re

A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'field_wall_drive_r520_v1'
read=lambda p:json.loads(Path(p).read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main(final=False):
    cfg=read(A/'config.json');before=read(OLD/'config.json')
    changed=sorted(k for k in set(cfg)|set(before) if cfg.get(k)!=before.get(k))
    assert changed==['launch','registered_at','scope'] and cfg==read(A/'expected_config.json')
    launch=dict(cfg['launch']);launch['entry_g1']=before['launch']['entry_g1'];assert launch==before['launch']
    assert read(A/'arguments.json')==[v.replace(OLD.name,A.name) for v in read(OLD/'arguments.json')]
    selection=read(A/'source_selection.json')
    assert all(selection[k] is final for k in ('freeze_authorized','wsl_exclusive_owner','drive_authorized'))
    assert selection['final_production_source_selection_pending'] is (not final)
    provenance=read(A/'recipe_provenance.json')
    for item in provenance['files']:
        assert sha(A/item['path'])==item['after_sha256'],item['path']
        if item['byte_exact']:assert sha(A/item['path'])==sha(OLD/item['path'])
    for p,h in provenance['original_evidence'].items():assert sha(ROOT/p)==h,p
    preserved=read(A/'r520_preservation.json')
    assert len(preserved['files'])==preserved['count']
    for p,h in preserved['files'].items():assert sha(ROOT/p)==h,p
    from world_resources import verify_resources
    from field_wall_geometry import verify_derived
    resources=verify_resources();field=verify_derived();equivalence=read(A/'geometry_equivalence.json')
    assert resources==read(A/'resource_check.json')
    assert equivalence['passed'] and equivalence['r530_world_sha256']==field['derived_sha256']
    assert equivalence['complete_world_identical_except_four_resource_texts']
    actor=read(A/'geometry_right_side.json');assert actor['passed'] and actor['actor_path_samples']==601
    assert read(A/'field_static_geometry.json')['passed']
    for p,h in actor['source_hashes'].items():assert sha(ROOT/p)==h,p
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    for p in [*A.rglob('*.sh'),*A.rglob('*.bash')]:assert b'\r' not in p.read_bytes(),p
    for name in ('PLAN.md','READY.md'):
        if (A/name).exists():
            assert not [p for p in re.findall(r'\]\(([^)#]+)',(A/name).read_text(encoding='utf-8')) if not (A/p).exists() and p!='preparation_receipt.json']
    run=(A/'run.py').read_text(encoding='utf-8');launch_text=(A/'launch/simulation.launch.py').read_text(encoding='utf-8')
    assert 'return original_obstacles(DERIVED)' in run
    assert launch_text.index("bound.arguments['source']=DERIVED")<launch_text.index('report=original(*bound.args,**bound.kwargs)')
    assert "assert collision_digest(destination)==info['walls_collision_sha256']" in launch_text
    assert 'resources=verify_runtime_resources(destination)' in launch_text
    assert not any('world_override:=' in x for x in read(A/'arguments.json'))
    # 이 목록은 준비 시점의 관측일 뿐 동결/최종 선택이 아니다. root가 아직 공통 코드를 작업 중이다.
    baseline=read(OLD/'prepared.json')['frozen_inputs']
    common={p:h for p,h in baseline.items() if p.startswith(('src/','scripts/','tests/','config/')) or 'old_signal_610_r326_v1/' in p}
    delta={p:dict(before_sha256=h,after_sha256=sha(ROOT/p)) for p,h in common.items() if sha(ROOT/p)!=h}
    if final:
        from prepare import verify_recipe
        verify_recipe()
    assert not any((A/n).exists() for n in ('prepared.json','sources.zip','registered.json','root_execution_review.json','run1','preflight','runtime_resource_receipt.json'))
    result=dict(passed=True,scope='Windows resource and unchanged-geometry recipe preparation; not production import or source freeze',
        config_changed_keys_vs_r520=changed,actor_facilities_physics_speed_stop_acceptance_unchanged=True,
        generated_world_sha256=field['derived_sha256'],wall_collisions=475,wall_visuals=475,
        static_actor601_and_initial_ego_reused=True,external_asset_references=resources['asset_reference_count'],
        unique_asset_files=resources['unique_asset_file_count'],asset_files_all_exist=True,
        all_r520_files_preserved=preserved['count'],evaluator_and_runtime_same_derived_path=True,
        final_production_source_selection_pending=not final,observed_common_delta_not_final_selection=delta,
        live_runtime_import_tested=False,package_plugin_loader_tested=False,WSL_used=False,frozen=False,
        simulator_started=False,actual_run_count=0)
    output=A/('final_preparation_receipt.json' if final else 'preparation_receipt.json')
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('passed','generated_world_sha256','wall_collisions','external_asset_references','unique_asset_files','all_r520_files_preserved','frozen','actual_run_count')}))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--final',action='store_true');main(parser.parse_args().final)
