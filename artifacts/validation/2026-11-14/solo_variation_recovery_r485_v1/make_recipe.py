"""r477 첫 변형의 검증된 recipe만 새 경로에 준비한다. freeze/WSL 실행 없음."""
from pathlib import Path
import ast,hashlib,json,shutil
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
OLD=A.parent/'solo_variations_r477_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    assert not (A/'recipe_provenance.json').exists()
    names=['solo_runner.py','solo_evaluation.py','solo_finalize.py','facility_geometry.py',
        'facilities_only.sdf','expected_header.json','audit_runtime.py','header_comparison.py',
        'course.launch.py','launch/demo.launch.py','launch/local_pursuit.launch.py',
        'prepare.py','variation_launch.py','launch_adapter.py','run.py','stop_owned.py',
        'launch/simulation.launch.py','execution/execute_once.sh','execution/stop_once.sh',
        'execution/observe.py','execution/check_owned_processes.py']
    records=[]
    for name in names:
        src=OLD/name;dst=A/name;dst.parent.mkdir(parents=True,exist_ok=True)
        text=src.read_text(encoding='utf-8')
        text=text.replace(OLD.relative_to(ROOT).as_posix(),A.relative_to(ROOT).as_posix())
        text=text.replace('R477','R485').replace('r477','r485')
        if name=='prepare.py':
            text=text.replace("    'wall_surface_inward10': {'lateral_m': 0., 'yaw_delta_deg': 0., 'wall_extra_width_m': .02},\n",'')
        if name.endswith('.sh'):
            text=text.replace('start_left20_yaw_plus3|wall_surface_inward10','start_left20_yaw_plus3')
        dst.write_text(text,encoding='utf-8',newline='\n')
        if name=='expected_header.json':shutil.copyfile(src,dst)
        records.append(dict(path=name,source=src.relative_to(ROOT).as_posix(),before_sha256=sha(src),after_sha256=sha(dst)))
    config=json.loads((OLD/'config.json').read_bytes())
    config['scope']='r485 one start_left20_yaw_plus3 recovery run; r477 config/physics/sensors/STOP retained; only approved r481/r482 Python source changes'
    config['launch']['entry_g1']=(A/'course.launch.py').relative_to(ROOT).as_posix()
    (A/'config.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    for name in ['connection_v1/start_left20_yaw_plus3.sdf','prepared/manifest.json']:
        dst=A/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(OLD/name,dst)
        records.append(dict(path=name,source=(OLD/name).relative_to(ROOT).as_posix(),before_sha256=sha(OLD/name),after_sha256=sha(dst)))
    policy=json.loads((A.parent/'solo_finish_r481_v1/changes.json').read_bytes())
    r482='src/arena_autonomy/arena_autonomy/current_lane_preference.py'
    stage=A.parent/'jetson_plan_cost_r482_v1/stage'/r482
    assert stage.is_file()
    original=json.loads((OLD/'execution_prepared.json').read_bytes())['frozen_inputs']
    policy.append(dict(path=r482,before_sha256=original[r482],after_sha256=sha(stage)))
    (A/'source_selection.json').write_text(json.dumps(dict(policy_changes=policy,
        noncontrol_changes=['scripts/jetson_access.py'],common_integration_confirmed=False,
        freeze_authorized=False,wsl_owner_confirmed=False,expected_native_sha256=original[
            'src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-312-x86_64-linux-gnu.so']),indent=2)+'\n',encoding='utf-8')
    records.append(dict(path='config.json',source=(OLD/'config.json').relative_to(ROOT).as_posix(),
        before_sha256=sha(OLD/'config.json'),after_sha256=sha(A/'config.json'),changed_keys=['scope','launch.entry_g1']))
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    (A/'recipe_provenance.json').write_text(json.dumps(dict(source_case='r477 start_left20_yaw_plus3',
        files=records,source_prepared_sha256=sha(OLD/'execution_prepared.json'),
        source_archive_sha256=sha(OLD/'execution_sources.zip'),source_frozen=False,
        wsl_executed=False,case_count=1,build_required_expected=False),indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(recipe_files=len(records),case_count=1,frozen=False,wsl_executed=False)))

if __name__=='__main__':main()
