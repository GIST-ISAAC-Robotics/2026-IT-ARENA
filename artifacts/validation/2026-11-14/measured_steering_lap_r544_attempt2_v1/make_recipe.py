"""r534 미실행 단독 레시피를 r544의 명시 실측 count 파생 YAML에 연결한다."""
import hashlib,json,shutil
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'field_wall_solo_drive_r534_v1'
VEHICLE='src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml'
EXPECTED='dcb8a7130f3eea4a474aff4602b86547bb7f700e6fecaff3049bb8dbd32a5021'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_bytes())
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
def main():
    assert not (A/'config.json').exists()
    assert sha(ROOT/VEHICLE)==EXPECTED
    skip={'make_recipe.py','PLAN.md','READY.md','check_recipe.py','preparation_receipt.json','collection_fixture_check.json'}
    files=[p for p in OLD.iterdir() if p.is_file() and p.name not in skip]
    files += [p for folder in ('launch','execution_v1') for p in (OLD/folder).iterdir() if p.suffix in ('.py','.sh','.bash')]
    copied=[]
    for p in files:
        rel=p.relative_to(OLD);dest=A/rel;assert not dest.exists();dest.parent.mkdir(parents=True,exist_ok=True)
        if p.suffix in ('.py','.json','.sh','.bash'):
            dest.write_text(p.read_text(encoding='utf-8').replace(OLD.name,A.name).replace('r534','r544'),encoding='utf-8',newline='\n')
        else:shutil.copyfile(p,dest)
        copied.append(dict(path=rel.as_posix(),source=p.relative_to(ROOT).as_posix(),source_sha256=sha(p),sha256=sha(dest)))
    args=read(A/'arguments.json');args[args.index('--vehicle-config')+1]=VEHICLE;write(A/'arguments.json',args)
    cfg=read(A/'config.json');cfg['scope']='r544 nominal actor0 lap; r530 first-straight geometry, measured count-derived steering only; wheel angles not measured'
    cfg['registered_at']='2026-11-14 Windows draft; root final source and WSL handover pending'
    cfg['vehicle_and_sensor_unchanged']='r475 vehicle/sensors/mass/speed unchanged except explicit measured-count-derived steering YAML: joint cap0.30666234131856146rad, central command0.28475436568260193rad; wheel angles CAD-derived, not measured'
    write(A/'config.json',cfg);write(A/'expected_config.json',cfg)
    selection=read(A/'source_selection.json');selection['source_baseline']='artifacts/validation/2026-11-14/field_wall_guard_relevance_drive_r539_v1'
    selection['status']='r539 collected; r540 native integration/final source handover pending; Windows draft only'
    selection['additional_sources']=[VEHICLE];selection['additional_source_hashes']={VEHICLE:EXPECTED};write(A/'source_selection.json',selection)
    write(A/'draft_copy_manifest.json',dict(source=OLD.relative_to(ROOT).as_posix(),files=copied,world_sha256=sha(A/'field_world.sdf'),
        vehicle_config=VEHICLE,vehicle_config_sha256=EXPECTED,WSL_used=False,frozen=False,actual_runs=0))
    print('r544 draft copied; no freeze/WSL/run')
if __name__=='__main__':main()
