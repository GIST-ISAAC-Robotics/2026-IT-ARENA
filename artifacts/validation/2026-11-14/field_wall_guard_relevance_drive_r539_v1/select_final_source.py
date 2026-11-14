"""r538 137회귀 완료/WSL 독점 인계에 따른 r539 최종 입력 선택. 주행은 별도."""
import ast,hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'field_wall_resource_drive_r530_v1';CANDIDATE=A.parent/'guard_memory_relevance_r538_v1'
GUARD='src/arena_autonomy/arena_autonomy/guard_memory_command.py'
GUARD_SHA='dc81494c5162d0e07851e835bee7dfc96fac303a857fe0c922da50959647b26d'
read=lambda p:json.loads(Path(p).read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    assert not any((A/n).exists() for n in ('prepared.json','sources.zip','registered.json','run1'))
    baseline=read(OLD/'prepared.json')['frozen_inputs'];selection=read(A/'source_selection.json')
    transfer=read(CANDIDATE/'common_transfer/transfer.json');tests=read(CANDIDATE/'review_tests/updated_tests_manifest.json')
    assert sha(ROOT/GUARD)==GUARD_SHA
    expected={r['path']:r for r in transfer};assert len(expected)==3
    for p,row in expected.items():assert baseline[p]==row['before'] and sha(ROOT/p)==row['after'],p
    for row in tests:assert baseline[row['path']]==row['original_sha256'] and sha(ROOT/row['path'])==row['updated_sha256']
    suites=ET.parse(CANDIDATE/'common_transfer/pytest.xml').getroot()
    assert len(suites.findall('.//testcase'))==137 and not suites.findall('.//failure') and not suites.findall('.//error') and not suites.findall('.//skipped')
    common={p:h for p,h in baseline.items() if p.startswith(('src/','scripts/','tests/','config/')) or 'old_signal_610_r326_v1/' in p}
    delta=[dict(path=p,before_sha256=h,after_sha256=sha(ROOT/p)) for p,h in common.items() if sha(ROOT/p)!=h]
    assert {r['path'] for r in delta}==set(expected)
    selection['control_changes']=[dict(**r,role='r538 root-integrated retained-memory braking-envelope relevance') for r in delta if r['path']==GUARD]
    selection['noncontrol_changes']=[r['path'] for r in tests]
    selection['reviewed_noncontrol_hashes']={r['path']:r['updated_sha256'] for r in tests}
    selection['additional_sources']=[];selection['additional_tests']=[];selection['additional_source_hashes']={};selection['additional_test_hashes']={}
    selection.update(freeze_authorized=True,wsl_exclusive_owner=True,drive_authorized=True,final_production_source_selection_pending=False,
        pending_integrations=[],status='r538 guard plus two approved tests selected; root137 regression and one-run exclusive handover complete',
        root_authorization='Root explicitly authorizes one r539 same-condition traffic run after SHA/import/freeze/competition0; no Jetson, no repeat')
    selection['included_integrations']=read(OLD/'source_selection.json')['included_integrations']+['r538 braking-envelope relevant retained-memory cap; 137 WSL tests passed']
    selection['excluded_integrations']=['r540 optimization stage not selected','Jetson prohibited','Unrelated hardware/servo preparations not invoked; no measured profile adoption']
    excluded={}
    for p,row in read(OLD/'source_selection.json').get('excluded_unrelated_new_files',{}).items():
        excluded[p]=dict(baseline_observed_sha256=row['sha256'],current_sha256=sha(ROOT/p),included_in_runtime=False,
            reason='Other-session hardware/operator preparation; snapshot identity only, no profile adoption')
    selection['excluded_unrelated_new_files']=excluded
    evidence=['common_transfer/transfer.json','common_transfer/pytest.xml','common_transfer/pytest.log','common_transfer/setup_error.txt','review_tests/updated_tests_manifest.json']
    selection['integration_evidence_sha256']={(CANDIDATE/p).relative_to(ROOT).as_posix():sha(CANDIDATE/p) for p in evidence}
    cfg=read(A/'config.json');old_cfg=read(OLD/'config.json');assert cfg==read(A/'expected_config.json')
    assert sorted(k for k in set(cfg)|set(old_cfg) if cfg.get(k)!=old_cfg.get(k))==['launch','registered_at','scope']
    launch=dict(cfg['launch']);launch['entry_g1']=old_cfg['launch']['entry_g1'];assert launch==old_cfg['launch']
    assert read(A/'arguments.json')==[v.replace(OLD.name,A.name) for v in read(OLD/'arguments.json')]
    for item in read(A/'recipe_provenance.json')['files']:assert sha(A/item['path'])==item['after_sha256'],item['path']
    assert sha(A/'field_world.sdf')==sha(OLD/'field_world.sdf')=='c32c19343d488a0e3d4909c733b1d1f37efcf2ea2822ccd7dff33c934a299f09'
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    (A/'source_selection.json').write_text(json.dumps(selection,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    result=dict(baseline_files=len(common),source_delta=delta,production_changes=1,approved_test_changes=2,unexpected_changes=[],
        guard_sha256=GUARD_SHA,root_regression_passed=137,measured_profile_adopted=False,
        unrelated_preparation_snapshot=excluded,world_unchanged=True,geometry_recalculated=False,source_frozen=False,simulator_started=False)
    (A/'final_source_manifest.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(production_changes=1,test_changes=2,unexpected_changes=0,root_regression_passed=137,guard_sha256=GUARD_SHA,measured_profile_adopted=False)))


if __name__=='__main__':main()
