"""r550 Windows 준비만 생성한다. 기존 주행/ZIP과 공통 소스는 변경하지 않는다."""
import ast,hashlib,json
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];BASE=A.parent/'measured_steering_lap_r544_attempt2_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_bytes())
def write(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
def main():
    assert not (A/'arguments.json').exists()
    frozen=read(BASE/'prepared.json');report=read(BASE/'run1/report.json');prefix=BASE.relative_to(ROOT).as_posix()+'/'
    assert sha(BASE/'sources.zip')==frozen['source_archive_sha256']
    assert report['shutdown_clean'] and not read(BASE/'execution_v1/owned_processes_after.json')['remaining_owned_work']
    names=['audit_runtime.py','collect_evidence.py','collect_result.py','course.launch.py','facility_geometry.py','field_wall_geometry.py',
        'header_comparison.py','launch_adapter.py','run.py','solo_evaluation.py','solo_finalize.py','solo_runner.py','steering_runtime_audit.py',
        'stop_owned.py','world_resources.py','arguments.json','config.json','expected_config.json','expected_header.json','field_geometry_audit.json',
        'field_static_geometry.json','geometry_right_side.json','resource_check.json','resource_manifest.json','facilities_only.sdf','field_world.sdf',
        'launch/demo.launch.py','launch/local_pursuit.launch.py','launch/simulation.launch.py','execution_v1/check_owned_processes.py',
        'execution_v1/observe.py','execution_v1/execute_once.sh','execution_v1/stop_once.sh','execution_v1/finish_collection.bash']
    copied=[]
    for name in names:
        source=BASE/name;target=A/name;assert sha(source)==frozen['frozen_inputs'][prefix+name];assert not target.exists()
        target.parent.mkdir(parents=True,exist_ok=True);data=source.read_bytes()
        if target.suffix in ('.py','.json','.sh','.bash'):data=data.decode('utf-8').replace(BASE.name,A.name).encode('utf-8')
        target.write_bytes(data);copied.append(dict(path=name,baseline_sha256=sha(source),sha256=sha(target),only_artifact_path_relocated=True))
    baseline={p:h for p,h in {**frozen['frozen_inputs'],**report['source_sha256']}.items() if not p.startswith(prefix)}
    allowed={
        'src/arena_autonomy/arena_autonomy/lidar_safety.py':'e21a9b3f2332b60173e7d6ac00f4e970438872ac2d67612e82cb92edc5cd4b13',
        'src/arena_autonomy/arena_autonomy/current_wall_reacquire.py':'db429c5aed73561a1e1e455513d81d73d12d015067127916a378da7a77cc9ef0'}
    selected={};delta=[]
    for p,before in baseline.items():
        after=sha(ROOT/p)
        if after!=before:assert allowed.get(p)==after,p;delta.append(dict(path=p,before_sha256=before,after_sha256=after))
        selected[p]=after
    assert {r['path'] for r in delta}==set(allowed)
    extra='src/arena_autonomy/arena_autonomy/guard_status_payload.py';selected[extra]=sha(ROOT/extra)
    assert selected[extra]=='54c2e97df27759f48dae417321c87ae14df7c598e5b45a3088cd87df824c51bb'
    write(A/'source_selection.json',dict(freeze_authorized=False,wsl_exclusive_owner=False,drive_authorized=False,
        final_production_source_selection_pending=True,source_baseline=BASE.relative_to(ROOT).as_posix(),required_sources=selected,
        source_delta=delta,additional_sources=[extra],status_detail='full',native_rebuild_authorized=False,
        status='Windows draft; root r549 regressions/final handover pending; no WSL/freeze/run'))
    write(A/'observational_baseline.json',frozen['observed_inputs'])
    preserved={name:sha(BASE/name) for name in ('prepared.json','sources.zip','run1/report.json','integrity.json','execution_v1/owned_processes_after.json')}
    write(A/'recipe_provenance.json',dict(baseline=BASE.relative_to(ROOT).as_posix(),preserved_evidence=preserved,files=copied,
        measured_vehicle_sha256='dcb8a7130f3eea4a474aff4602b86547bb7f700e6fecaff3049bb8dbd32a5021',
        world_sha256=sha(A/'field_world.sdf'),expected_header_sha256=sha(A/'expected_header.json'),
        seven_angle_audit_reused=True,static_geometry_receipts_reused=True,original_acceptance_speed_stop_unchanged=True))
    print(json.dumps(dict(copied_files=len(copied),planned_required_external_inputs=len(selected),source_delta=delta,new_module=extra,
        baseline_archive_sha256=preserved['sources.zip'],new_archive_created=False,WSL_used=False,actual_runs=0)))
if __name__=='__main__':main()
