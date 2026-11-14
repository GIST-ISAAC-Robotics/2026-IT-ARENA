"""r515 실행 레시피를 r520 artifact에 복사한다. 동결·WSL·주행은 하지 않는다."""
import ast
import hashlib
import json
from pathlib import Path
import shutil

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
OLD=A.parent/'current_guide_extent_drive_r515_v1'
OLDER='continuous_pass_preview_drive_r513_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_bytes())
EXACT=['expected_header.json','facilities_only.sdf','facility_geometry.py','goal_eval.py',
       'course.launch.py','launch_adapter.py','launch/demo.launch.py','launch/local_pursuit.launch.py','.gitignore']
LOCAL=['run.py','preflight.py','collect_result.py','assess.py','stop_owned.py','prepare.py','prepare_once.sh',
       'launch/simulation.launch.py','execution_v1/execute_once.sh','execution_v1/stop_once.sh',
       'execution_v1/observe.py','execution_v1/check_owned_processes.py','execution_v1/finish_collection.bash']


def save(path,text):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8',newline='\n') as f:f.write(text)


def main():
    assert not (A/'source_selection.json').exists()
    copied=[]
    for name in EXACT+LOCAL:
        dest=A/name;dest.parent.mkdir(parents=True,exist_ok=True)
        if name in EXACT:
            assert not dest.exists();shutil.copyfile(OLD/name,dest)
        else:
            content=(OLD/name).read_text(encoding='utf-8').replace(OLD.name,A.name).replace('r515','r520')
            if name=='prepare.py':
                content=content.replace(OLDER,OLD.name).replace('r513','r515')
            if name=='run.py':
                content=content.replace('from launch_adapter import instrument_start',
                    'from launch_adapter import instrument_start\nfrom field_wall_geometry import DERIVED, SOURCE_SHA, verify_derived')
                content=content.replace('original_obstacles = target.obstacle_polygons',
                    'verify_derived()\n    original_obstacles = target.obstacle_polygons\n    def field_obstacles(path):\n        assert sha(Path(path)) == SOURCE_SHA\n        return original_obstacles(DERIVED)')
                content=content.replace('target.obstacle_polygons = lambda path: [*original_obstacles(path), *shapes]',
                    'target.obstacle_polygons = lambda path: [*field_obstacles(path), *shapes]')
            if name=='launch/simulation.launch.py':
                content=content.replace('import json\n','import json\nimport inspect\nimport sys\n')
                content=content.replace("original=module._runtime_world_overrides", "original=module._runtime_world_overrides\nsys.path.insert(0,str(A))\nfrom field_wall_geometry import DERIVED,SOURCE_SHA,verify_derived,collision_digest")
                content=content.replace('    report=original(*args,**kwargs)',
                    "    info=verify_derived()\n    bound=inspect.signature(original).bind(*args,**kwargs)\n    assert hashlib.sha256(Path(bound.arguments['source']).read_bytes()).hexdigest()==SOURCE_SHA\n    bound.arguments['source']=DERIVED\n    report=original(*bound.args,**bound.kwargs)")
                content=content.replace('    original_bytes=destination.read_bytes();root=ET.fromstring(original_bytes)',
                    "    assert collision_digest(destination)==info['walls_collision_sha256']\n    (A/'field_runtime_receipt.json').write_text(json.dumps(dict(derived_sha256=info['derived_sha256'],runtime_walls_collision_sha256=collision_digest(destination),same_derived_world_for_evaluation=True,world_override_used=False,source_injected_before_visual_batch=True),indent=2)+'\\n')\n    original_bytes=destination.read_bytes();root=ET.fromstring(original_bytes)")
            save(dest,content)
        copied.append(dict(path=name,source=(OLD/name).relative_to(ROOT).as_posix(),before_sha256=sha(OLD/name),after_sha256=sha(dest),byte_exact=name in EXACT))
    cfg=read(OLD/'config.json')
    cfg['scope']='r520 unmeasured field-wall sensitivity: selected two walls, y3..11.5m, same-phase 30mm/1m, 5mm thick, 20mm joints each600mm; same actor/facilities/12sim s'
    cfg['registered_at']='2026-11-14 r520 preparation only, not frozen or registered execution'
    cfg['launch']['entry_g1']=(A/'course.launch.py').relative_to(ROOT).as_posix()
    for name in ('config.json','expected_config.json'):save(A/name,json.dumps(cfg,ensure_ascii=False,indent=2)+'\n')
    save(A/'arguments.json',json.dumps([x.replace(OLD.name,A.name) for x in read(OLD/'arguments.json')],indent=2)+'\n')
    selection=dict(control_changes=[],noncontrol_changes=[],additional_sources=[],additional_tests=[],
        freeze_authorized=False,wsl_exclusive_owner=False,drive_authorized=False,
        final_production_source_selection_pending=True,
        included_integrations=read(OLD/'source_selection.json')['included_integrations'],
        excluded_integrations=['r518/r521 integration awaits root decision','Jetson/hardware/sensors untouched'],
        status='Generated geometry and Windows static preparation only; final source selection and execution await root',
        root_authorization='2026-11-14 root approved geometry generation/static601+initial-ego checks only; no freeze or run',
        field_geometry='field_geometry_audit.json')
    save(A/'source_selection.json',json.dumps(selection,ensure_ascii=False,indent=2)+'\n')
    original={**read(OLD/'recipe_provenance.json')['original_evidence']}
    for name in ('RESULT.md','prepared.json','integrity.json','assessment_summary.json','execution_summary.json','geometry_right_side.json'):
        original[(OLD/name).relative_to(ROOT).as_posix()]=sha(OLD/name)
    provenance=dict(source='r515 tools, with artifact-local shared derived-wall source injection',files=copied,
        original_evidence=original,threshold_changes=False,case_actor_acceptance_unchanged=True,
        geometry_reused=False,geometry_recalculation_required=True,common_source_selection_pending=True,
        one_execution_only=True,repeat_on_failure=False,actual_simulation_started=False)
    save(A/'recipe_provenance.json',json.dumps(provenance,ensure_ascii=False,indent=2)+'\n')
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    for p in [*A.rglob('*.sh'),*A.rglob('*.bash')]:assert b'\r' not in p.read_bytes()
    print(json.dumps(dict(recipe_files=len(copied),frozen=False,WSL_used=False,simulation_started=False)))


if __name__=='__main__':main()
