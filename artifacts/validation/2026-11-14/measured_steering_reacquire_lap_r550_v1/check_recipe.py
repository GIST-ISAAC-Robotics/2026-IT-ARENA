"""Windows 준비 조건/해시만 확인한다. ROS/WSL/제어기 실행 없음."""
import ast,hashlib,json,re
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];BASE=A.parent/'measured_steering_lap_r544_attempt2_v1'
read=lambda p:json.loads(Path(p).read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    provenance=read(A/'recipe_provenance.json');selection=read(A/'source_selection.json')
    for item in provenance['files']:
        assert sha(A/item['path'])==item['sha256']
        assert sha(BASE/item['path'])==item['baseline_sha256']
    for name,h in provenance['preserved_evidence'].items():assert sha(BASE/name)==h
    for p,h in selection['required_sources'].items():assert sha(ROOT/p)==h,p
    assert not any(selection[k] for k in ('freeze_authorized','wsl_exclusive_owner','drive_authorized'))
    assert selection['final_production_source_selection_pending'] and selection['status_detail']=='full'
    args=read(A/'arguments.json');assert args==[v.replace(BASE.name,A.name) for v in read(BASE/'arguments.json')]
    cfg=read(A/'config.json');assert cfg==read(A/'expected_config.json')
    assert json.loads(json.dumps(cfg).replace(A.name,BASE.name))==read(BASE/'config.json')
    assert cfg['cases'][0]['duration_s']==80 and cfg['cases'][0]['schedule']==[]
    assert sha(A/'field_world.sdf')==provenance['world_sha256']==sha(BASE/'field_world.sdf')
    assert sha(A/'expected_header.json')==sha(BASE/'expected_header.json')
    assert read(A/'expected_header.json')['follow_config']['max_steering_rad']==.28475436568260193
    audit=(A/'steering_runtime_audit.py').read_text().replace(A.name,BASE.name)
    assert audit==(BASE/'steering_runtime_audit.py').read_text()
    tree=ast.parse((ROOT/'src/arena_autonomy/arena_autonomy/lidar_safety.py').read_bytes())
    defaults=[node.args[1].value for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)
        and node.func.attr=='declare_parameter' and len(node.args)>=2 and isinstance(node.args[0],ast.Constant) and node.args[0].value=='safety_status_detail']
    assert defaults==['full'];assert not any('safety_status_' in arg for arg in args)
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    for p in [*A.rglob('*.sh'),*A.rglob('*.bash')]:assert b'\r' not in p.read_bytes()
    assert not any((A/name).exists() for name in ('prepared.json','sources.zip','registered.json','run1','preflight','root_execution_review.json'))
    from prepare import verify_recipe
    rejected=False
    try:verify_recipe()
    except AssertionError:rejected=True
    assert rejected
    receipt=dict(passed=True,scope='Windows preparation only; hashes are candidate input preview, not actual execution',
        source_selection_sha256=sha(A/'source_selection.json'),planned_required_external_inputs=len(selection['required_sources']),
        planned_external_input_map_sha256=hashlib.sha256(json.dumps(selection['required_sources'],sort_keys=True,separators=(',',':')).encode()).hexdigest(),
        baseline_source_archive_sha256=provenance['preserved_evidence']['sources.zip'],new_source_archive_created=False,
        actual_execution_input_hash=None,world_sha256=sha(A/'field_world.sdf'),vehicle_sha256=provenance['measured_vehicle_sha256'],
        expected_header_sha256=sha(A/'expected_header.json'),steering_runtime_audit_sha256=sha(A/'steering_runtime_audit.py'),seven_angle_checks_unchanged=True,
        arguments=args,arguments_sha256=sha(A/'arguments.json'),source_delta=selection['source_delta'],additional_sources=selection['additional_sources'],
        actor_count=0,facility_count=3,max_sim_seconds=80,no_progress_stop_sim_seconds=8,real_time_factor=.25,status_detail='full',
        original_conditions_unchanged=True,unauthorized_freeze_rejected=True,previous_raw_and_archive_unchanged=True,WSL_used=False,frozen=False,actual_runs=0)
    (A/'preparation_receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:receipt[k] for k in ('passed','planned_required_external_inputs','source_selection_sha256','planned_external_input_map_sha256','world_sha256','expected_header_sha256','WSL_used','actual_runs')}))
if __name__=='__main__':main()
