"""r530 실제 결과 검토 후 최종 소스를 선택한다. 현 준비 단계에서는 동결할 수 없다."""
import argparse,ast,hashlib,json,zipfile
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3];TRAFFIC=A.parent/'field_wall_resource_drive_r530_v1'
SOLO=A.parent/'solo_full_lap_r475_v1'
read=lambda p:json.loads(Path(p).read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()


def write_new(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n')


def arguments():return read(A/'arguments.json')


def verify_traffic_review():
    review=read(A/'traffic_review.json')
    assert review['root_reviewed'] and review['solo_authorized_after_traffic_review']
    assert review['traffic_report_sha256']==sha(TRAFFIC/'run1/report.json')
    assert review['traffic_integrity_sha256']==sha(TRAFFIC/'integrity.json')
    assert review['traffic_archive_sha256']==sha(TRAFFIC/'sources.zip')
    report=read(TRAFFIC/'run1/report.json');integrity=read(TRAFFIC/'integrity.json')
    assert review['original_traffic_passed'] is report['passed']  # 교통 실패를 소급하여 성공시키지 않는다.
    assert review['traffic_normal_shutdown_and_pid0_confirmed'] and report['shutdown_clean']
    assert not read(TRAFFIC/'execution_v1/owned_processes_after.json')['remaining_owned_work']
    assert not integrity['changed_inputs'] and integrity['archive_unchanged']
    assert (TRAFFIC/'execution_v1/run.exit').exists()
    return review


def verify_recipe():
    selection=read(A/'source_selection.json')
    assert all(selection[k] for k in ('freeze_authorized','wsl_exclusive_owner','drive_authorized'))
    assert not selection['final_production_source_selection_pending']
    verify_traffic_review()
    baseline=read(TRAFFIC/'prepared.json');assert sha(TRAFFIC/'sources.zip')==baseline['source_archive_sha256']
    cfg=read(A/'config.json');assert cfg==read(A/'expected_config.json')
    old_cfg=read(SOLO/'config.json')
    assert cfg['acceptance']==old_cfg['acceptance'] and cfg['cases']==old_cfg['cases']
    launch=dict(cfg['launch']);launch['entry_g1']=old_cfg['launch']['entry_g1'];assert launch==old_cfg['launch']
    assert arguments()==[v.replace(SOLO.name,A.name) for v in read(SOLO/'prepared.json')['args']]
    provenance=read(A/'recipe_provenance.json')
    for item in provenance['files']:assert sha(A/item['path'])==item['sha256'],item['path']
    common={p:h for p,h in baseline['frozen_inputs'].items() if p.startswith(('src/','scripts/','tests/','config/')) or 'old_signal_610_r326_v1/' in p}
    allowed={item['path']:item for item in selection['control_changes']};selected={};delta=[]
    for p,before in common.items():
        after=sha(ROOT/p)
        if p in allowed:assert before==allowed[p]['before_sha256'] and after==allowed[p]['after_sha256'],p
        elif before!=after:assert p in selection['noncontrol_changes'],p
        if before!=after:delta.append(dict(path=p,before_sha256=before,after_sha256=after))
        selected[p]=after
    assert set(allowed)<={r['path'] for r in delta}
    for p in [*selection['additional_sources'],*selection['additional_tests']]:
        assert p not in selected;selected[p]=sha(ROOT/p)
    for p,h in {**selection['reviewed_noncontrol_hashes'],**selection['additional_source_hashes'],**selection['additional_test_hashes']}.items():assert selected[p]==h,p
    from field_wall_geometry import verify_derived
    from world_resources import verify_resources
    assert verify_derived()['static_geometry_passed'] and sha(A/'field_world.sdf')==sha(TRAFFIC/'field_world.sdf')
    for item in verify_resources()['asset_references']:selected[item['repository_path']]=item['sha256']
    native='src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-312-x86_64-linux-gnu.so'
    assert selected[native]==common[native]
    return dict(source_delta=delta,native_sha256=selected[native],args=arguments()),selected


def freeze():
    assert not any((A/p).exists() for p in ('prepared.json','sources.zip','registered.json','run1'))
    info,selected=verify_recipe();assert read(A/'preflight/result.json')['passed']
    for p,h in read(A/'recipe_provenance.json')['original_evidence'].items():
        assert sha(ROOT/p)==h,p;selected[p]=h
    for p in [*A.glob('*.py'),*A.glob('*.json'),*A.glob('*.sdf'),*A.glob('*.md'),*A.glob('*.sh'),
              *(A/'launch').glob('*.py'),*(A/'execution_v1').glob('*.py'),*(A/'execution_v1').glob('*.sh'),
              *(A/'execution_v1').glob('*.bash'),A/'preflight/result.json']:
        if p.suffix=='.py':ast.parse(p.read_bytes())
        selected[p.relative_to(ROOT).as_posix()]=sha(p)
    for p in (TRAFFIC/'prepared.json',TRAFFIC/'sources.zip',TRAFFIC/'run1/report.json',TRAFFIC/'integrity.json',TRAFFIC/'execution_v1/owned_processes_after.json'):
        selected[p.relative_to(ROOT).as_posix()]=sha(p)
    with zipfile.ZipFile(A/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in selected:
            if not p.endswith('.zip'):z.write(ROOT/p,p)
    value=dict(**info,frozen_inputs=selected,source_archive_sha256=sha(A/'sources.zip'),
        actor_count=0,facilities=3,max_sim_seconds=80,real_time_factor=.25,no_progress_stop_sim_seconds=8,
        native_rebuilt=False,one_execution_only=True,repeat_on_failure=False,traffic_result_reviewed=True)
    write_new(A/'prepared.json',value)
    return dict(inputs=len(selected),prepared_sha256=sha(A/'prepared.json'),source_archive_sha256=value['source_archive_sha256'])


def verify_prepared():
    data=read(A/'prepared.json')
    changed=[p for p,h in data['frozen_inputs'].items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
    assert not changed,changed
    assert sha(A/'sources.zip')==data['source_archive_sha256']
    return data


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--freeze',action='store_true');parser.add_argument('--check',action='store_true');args=parser.parse_args()
    print(json.dumps(freeze() if args.freeze else dict(checked=True,inputs=len(verify_prepared()['frozen_inputs'])) if args.check else verify_recipe()[0]))
