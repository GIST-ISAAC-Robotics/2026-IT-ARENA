"""동일 조건/SHA와 미승인 상태만 검사. 기하601재계산·WSL·제어 실행 없음."""
import ast,hashlib,json,re
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'field_wall_resource_drive_r530_v1'
read=lambda p:json.loads(p.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    selection=read(A/'source_selection.json')
    assert not any(selection[k] for k in ('freeze_authorized','wsl_exclusive_owner','drive_authorized'))
    assert selection['final_production_source_selection_pending']
    assert selection['expected_control_change_paths']==['src/arena_autonomy/arena_autonomy/guard_memory_command.py']
    cfg=read(A/'config.json');old=read(OLD/'config.json')
    changed=sorted(k for k in set(cfg)|set(old) if cfg.get(k)!=old.get(k));assert changed==['launch','registered_at','scope']
    launch=dict(cfg['launch']);launch['entry_g1']=old['launch']['entry_g1'];assert launch==old['launch']
    assert cfg==read(A/'expected_config.json') and read(A/'arguments.json')==[v.replace(OLD.name,A.name) for v in read(OLD/'arguments.json')]
    provenance=read(A/'recipe_provenance.json')
    for item in provenance['files']:
        assert sha(A/item['path'])==item['after_sha256'] and sha(ROOT/item['source'])==item['before_sha256'],item['path']
        if item['byte_exact']:assert item['before_sha256']==item['after_sha256']
    for p,h in provenance['original_evidence'].items():assert sha(ROOT/p)==h,p
    scene=read(A/'same_scene_receipt.json');assert scene['world_byte_identical'] and scene['world_sha256']==sha(A/'field_world.sdf')
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    for p in [*A.rglob('*.sh'),*A.rglob('*.bash')]:assert b'\r' not in p.read_bytes(),p
    for p in A.glob('*.md'):
        assert not [v for v in re.findall(r'\]\(([^)#]+)',p.read_text(encoding='utf-8')) if not (A/v).exists() and v!='preparation_receipt.json']
    assert not any((A/n).exists() for n in ('prepared.json','sources.zip','registered.json','run1','preflight','root_execution_review.json'))
    from prepare import verify_recipe
    refused=False
    try:verify_recipe()
    except AssertionError:refused=True
    assert refused
    result=dict(passed=True,config_changed_keys=changed,same_world_sha256=scene['world_sha256'],same_475_wall_geometry=True,
        same_actor_facilities_physics_speed_resources=True,same_traffic_runner_and_recorder=True,baseline='r530 failed final_gap',
        baseline_guard_sha256=read(OLD/'prepared.json')['frozen_inputs']['src/arena_autonomy/arena_autonomy/guard_memory_command.py'],
        expected_control_change_paths=selection['expected_control_change_paths'],geometry_recalculated=False,static601_reused=True,
        original_verdict_preserved=True,unauthorized_freeze_rejected=True,source_selection_pending=True,WSL_used=False,frozen=False,actual_run_count=0)
    (A/'preparation_receipt.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
