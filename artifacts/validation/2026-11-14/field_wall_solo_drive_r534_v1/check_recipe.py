"""Windows 준비 검증만 수행한다. ROS/WSL/제어기/시뮬레이터 실행 없음."""
import ast,hashlib,json,re
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];SOLO=A.parent/'solo_full_lap_r475_v1';TRAFFIC=A.parent/'field_wall_resource_drive_r530_v1'
read=lambda p:json.loads(p.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    provenance=read(A/'recipe_provenance.json')
    for item in provenance['files']:
        assert sha(A/item['path'])==item['sha256'],item['path']
        assert sha(ROOT/item['source'])==item['source_sha256'],item['source']
    for p,h in provenance['original_evidence'].items():assert sha(ROOT/p)==h,p
    cfg=read(A/'config.json');baseline=read(SOLO/'config.json')
    changed=sorted(k for k in set(cfg)|set(baseline) if cfg.get(k)!=baseline.get(k));assert changed==['launch','registered_at','scope']
    launch=dict(cfg['launch']);launch['entry_g1']=baseline['launch']['entry_g1'];assert launch==baseline['launch']
    assert cfg==read(A/'expected_config.json') and cfg['cases'][0]['duration_s']==80 and cfg['cases'][0]['schedule']==[]
    assert read(A/'arguments.json')==[v.replace(SOLO.name,A.name) for v in read(SOLO/'prepared.json')['args']]
    selection=read(A/'source_selection.json')
    assert not any(selection[k] for k in ('freeze_authorized','wsl_exclusive_owner','drive_authorized'))
    assert selection['final_production_source_selection_pending']
    from field_wall_geometry import verify_derived,collision_digest,walls_element
    from world_resources import verify_resources
    from prepare_safety_capture import instrument
    field=verify_derived();resources=verify_resources()
    assert sha(A/'field_world.sdf')==sha(TRAFFIC/'field_world.sdf')==field['derived_sha256']
    assert sha(A/'facilities_only.sdf')==sha(TRAFFIC/'facilities_only.sdf')==sha(SOLO/'facilities_only.sdf')
    assert len(walls_element(A/'field_world.sdf').findall('collision'))==475
    assert len(walls_element(A/'field_world.sdf').findall('visual'))==475
    assert (A/'solo_runner.py').read_text(encoding='utf-8')==instrument((SOLO/'solo_runner.py').read_text(encoding='utf-8'))
    assert read(A/'geometry_right_side.json')['initial_ego']['wall_overlap_m2']==0
    for p,h in read(A/'geometry_right_side.json')['source_hashes'].items():assert sha(ROOT/p)==h,p
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    for p in [*A.rglob('*.sh'),*A.rglob('*.bash')]:assert b'\r' not in p.read_bytes(),p
    for p in A.glob('*.md'):
        assert not [v for v in re.findall(r'\]\(([^)#]+)',p.read_text(encoding='utf-8')) if not (A/v).exists() and v!='preparation_receipt.json']
    assert not any((A/n).exists() for n in ('prepared.json','sources.zip','registered.json','run1','preflight','traffic_review.json','root_execution_review.json'))
    from prepare import verify_recipe
    rejected=False
    try:verify_recipe()
    except AssertionError:rejected=True
    assert rejected,'authorization gate must reject before absent r530 results are consulted'
    result=dict(passed=True,scope='Windows preparation only',config_changed_keys=changed,actor_count=0,facilities=3,
        max_sim_seconds=80,no_progress_stop_sim_seconds=8,nominal_grid_slot=3,spawn_variation_added=False,
        speed_profile=cfg['launch']['speed_profile'],real_time_factor=.25,original_solo_acceptance_and_stop_unchanged=True,
        world_byte_identical_to_r530=True,world_sha256=field['derived_sha256'],walls_collision_sha256=collision_digest(A/'field_world.sdf'),
        wall_collisions=475,wall_visuals=475,external_asset_references=resources['asset_reference_count'],unique_asset_files=resources['unique_asset_file_count'],
        initial_ego_static_evidence_reused=True,source_before_run_is_future_r530_frozen_selection=True,
        traffic_result_review_required=True,unauthorized_freeze_rejected=True,safety_capture_logger_only_reversible=True,
        raw_report_unchanged_by_collector=True,final_source_selection_pending=True,WSL_used=False,frozen=False,actual_run_count=0)
    (A/'preparation_receipt.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
