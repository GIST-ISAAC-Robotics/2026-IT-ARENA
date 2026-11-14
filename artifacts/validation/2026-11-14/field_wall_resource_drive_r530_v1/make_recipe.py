"""r520 실패를 보존하며 r530 경로 보정 및 대기 중인 실행 레시피만 준비한다."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
from world_resources import create,verify_resources,canonical,geometry_digest,inventory

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
OLD=A.parent/'field_wall_drive_r520_v1'
OLDER='current_guide_extent_drive_r515_v1'
read=lambda p:json.loads(p.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
EXACT=['expected_header.json','facilities_only.sdf','facility_geometry.py','goal_eval.py',
       'course.launch.py','launch_adapter.py','launch/demo.launch.py','launch/local_pursuit.launch.py','.gitignore',
       'geometry_right_side.json','field_static_geometry.json']
LOCAL=['run.py','preflight.py','collect_result.py','assess.py','stop_owned.py','prepare.py','prepare_once.sh',
       'launch/simulation.launch.py','execution_v1/execute_once.sh','execution_v1/stop_once.sh',
       'execution_v1/observe.py','execution_v1/check_owned_processes.py','execution_v1/finish_collection.bash']


def save(path,text):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8',newline='\n') as f:f.write(text)


def write(path,data):save(path,json.dumps(data,ensure_ascii=False,indent=2)+'\n')


def main():
    assert not (A/'source_selection.json').exists()
    # 원실패의 모든 파일(원시 입력 포함)을 해시로 보존 확인한다. 다시 실행하지 않는다.
    old_files={p.relative_to(ROOT).as_posix():sha(p) for p in OLD.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    create()
    copied=[]
    for name in EXACT+LOCAL:
        dest=A/name;dest.parent.mkdir(parents=True,exist_ok=True)
        if name in EXACT:
            assert not dest.exists();shutil.copyfile(OLD/name,dest)
        else:
            content=(OLD/name).read_text(encoding='utf-8')
            # OLD의 기준은 r515였고 새 기준은 r520이다. 서로 뒤섞이지 않게 치환한다.
            content=content.replace(OLD.name,'__R530_ARTIFACT__').replace(OLDER,OLD.name).replace('__R530_ARTIFACT__',A.name)
            content=content.replace('r520','r530')
            # 위 짧은 문서용 치환에서 기준 디렉터리 이름이 바뀌지 않도록 복원한다.
            content=content.replace('field_wall_drive_r530_v1',OLD.name)
            if name=='prepare.py':
                content=content.replace('r515 기준에 root 승인 공통 후보와 r530 파생 벽을 반영해 새 실행으로 동결한다.',
                    'r520 기준에 root 최종 승인 후보와 r530 자원 경로만 반영한다. 승인 전 동결 금지.')
            if name=='preflight.py':
                content=content.replace("assert sha(DERIVED)=='9bf5ba7491153b8ef045a96e2428f44bffc9d308f4d3431c25bee1f30ee166a2'",
                    "assert sha(DERIVED)==read(A/'resource_manifest.json')['output_world_sha256']")
            if name=='launch/simulation.launch.py':
                content=content.replace('from field_wall_geometry import DERIVED,SOURCE_SHA,verify_derived,collision_digest',
                    'from field_wall_geometry import DERIVED,SOURCE_SHA,verify_derived,collision_digest\nfrom world_resources import verify_runtime_resources')
                content=content.replace('    destination.write_bytes(after)',
                    "    destination.write_bytes(after)\n    resources=verify_runtime_resources(destination)\n    (A/'runtime_resource_receipt.json').write_text(json.dumps(resources,indent=2)+'\\n')")
            save(dest,content)
        copied.append(dict(path=name,source=(OLD/name).relative_to(ROOT).as_posix(),before_sha256=sha(OLD/name),after_sha256=sha(dest),byte_exact=name in EXACT))
    cfg=read(OLD/'config.json')
    cfg['scope']='r530: r520 geometry and original12sim s conditions retained; four texture paths resolved against original world; final controller selection pending'
    cfg['registered_at']='2026-11-14 r530 preparation only; no source freeze or run authorization'
    cfg['launch']['entry_g1']=(A/'course.launch.py').relative_to(ROOT).as_posix()
    for name in ('config.json','expected_config.json'):write(A/name,cfg)
    write(A/'arguments.json',[x.replace(OLD.name,A.name) for x in read(OLD/'arguments.json')])
    write(A/'source_selection.json',dict(control_changes=[],noncontrol_changes=[],additional_sources=[],additional_tests=[],
        freeze_authorized=False,wsl_exclusive_owner=False,drive_authorized=False,final_production_source_selection_pending=True,
        included_integrations=read(OLD/'source_selection.json')['included_integrations'],
        pending_integrations=['r526 delivery','r528 guard','r529 warmup; root final selection and SHA receipt required'],
        excluded_integrations=['Jetson/hardware/sensors untouched; WSL remains root-owned'],
        status='Resource paths and Windows-only recipe prepared; final production sources not selected',
        root_authorization='Root approved r530 resource correction and same12sim s recipe preparation only; no freeze/WSL/run',
        field_geometry='field_geometry_audit.json'))
    resources=verify_resources();write(A/'resource_check.json',resources)
    old_root=ET.parse(OLD/'field_world.sdf').getroot();new_root=ET.parse(A/'field_world.sdf').getroot()
    restored=copy.deepcopy(new_root)
    for item,(_,node,attribute,_) in zip(read(A/'resource_manifest.json')['changes'],inventory(restored)[0]):
        assert attribute is None;node.text=item['before']
    assert canonical(restored)==canonical(old_root)
    from field_wall_geometry import collision_digest,walls_element
    assert collision_digest(A/'field_world.sdf')==collision_digest(OLD/'field_world.sdf')
    walls=walls_element(A/'field_world.sdf')
    assert len(walls.findall('collision'))==len(walls.findall('visual'))==475
    audit=read(OLD/'field_geometry_audit.json')
    audit.update(derived_world=(A/'field_world.sdf').relative_to(ROOT).as_posix(),derived_sha256=sha(A/'field_world.sdf'),
        path_only_update_from_r520=True,resource_manifest='resource_manifest.json',
        non_wall_subtrees_unchanged=False,non_wall_subtrees_unchanged_except_four_texture_paths=True,
        geometry_receipts_reused_from=(OLD.relative_to(ROOT).as_posix()),
        geometry_receipts_reuse_reason='Canonical complete world equality after restoring four albedo paths; collision/visual/pose/surface/physics unchanged')
    write(A/'field_geometry_audit.json',audit)
    equivalence=dict(passed=True,r520_world_sha256=sha(OLD/'field_world.sdf'),r530_world_sha256=sha(A/'field_world.sdf'),
        complete_world_identical_except_four_resource_texts=True,wall_collisions=475,wall_visuals=475,
        walls_collision_sha256=collision_digest(A/'field_world.sdf'),
        tags={tag:dict(equal=geometry_digest(old_root,tag)==geometry_digest(new_root,tag),sha256=geometry_digest(new_root,tag))
              for tag in ('geometry','collision','pose','surface','physics')},
        facilities_byte_identical=sha(A/'facilities_only.sdf')==sha(OLD/'facilities_only.sdf'),
        support_mesh_sha256=resources['asset_references'][-1]['sha256'],
        actor_config_and_initial_condition_unchanged=True,static_actor601_and_initial_ego_reused=True,
        reused_receipts={n:dict(path=(A/n).relative_to(ROOT).as_posix(),sha256=sha(A/n),original_world_hash_retained=True)
            for n in ('geometry_right_side.json','field_static_geometry.json')},
        unchanged_original_walls=127,WSL_used=False,simulator_started=False)
    assert all(item['equal'] for item in equivalence['tags'].values())
    write(A/'geometry_equivalence.json',equivalence)
    original={**read(OLD/'recipe_provenance.json')['original_evidence']}
    # 작은 전제 파일만 새 실행 ZIP에 포함. 큰 원시 실패 파일은 별도 보존 감사에서 참조한다.
    for name in ('RESULT.md','prepared.json','integrity.json','assessment_summary.json','execution_summary.json',
                 'field_world.sdf','field_geometry_audit.json','geometry_right_side.json','field_static_geometry.json'):
        original[(OLD/name).relative_to(ROOT).as_posix()]=sha(OLD/name)
    for ref in resources['asset_references']:original[ref['repository_path']]=ref['sha256']
    write(A/'recipe_provenance.json',dict(source='r520 tools; four asset paths corrected, same physical geometry',files=copied,
        original_evidence=original,threshold_changes=False,case_actor_acceptance_unchanged=True,
        geometry_reused=True,geometry_recalculation_required=False,common_source_selection_pending=True,
        one_execution_only=True,repeat_on_failure=False,actual_simulation_started=False))
    assert all((ROOT/p).is_file() and sha(ROOT/p)==h for p,h in old_files.items())
    write(A/'r520_preservation.json',dict(passed=True,files=old_files,count=len(old_files),
        failed_original_run_unchanged=True,raw_inputs_included=True,scope='Hash comparison before and after r530 preparation'))
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    for p in [*A.rglob('*.sh'),*A.rglob('*.bash')]:assert b'\r' not in p.read_bytes()
    print(json.dumps(dict(recipe_files=len(copied),external_asset_references=resources['asset_reference_count'],
        unique_asset_files=resources['unique_asset_file_count'],walls=475,preserved_r520_files=len(old_files),
        frozen=False,WSL_used=False,simulation_started=False)))


if __name__=='__main__':main()
