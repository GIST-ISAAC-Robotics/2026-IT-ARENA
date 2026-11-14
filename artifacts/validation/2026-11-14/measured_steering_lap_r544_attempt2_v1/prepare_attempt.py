"""주행 전 감사 이름 오류만 수정한 별도 시도. 기존 실패/ZIP을 보존한다."""
import ast,hashlib,json,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'measured_steering_lap_r544_v1'
read=lambda p:json.loads(Path(p).read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
def main():
    assert not (A/'prepared.json').exists()
    old=read(OLD/'prepared.json');assert sha(OLD/'sources.zip')==old['source_archive_sha256']
    assert not read(OLD/'execution_v1/owned_processes_after.json')['remaining_owned_work']
    report=read(OLD/'run1/report.json');assert report['shutdown_audit']['started_child_count']==0 and not report['passed']
    oldprefix=OLD.relative_to(ROOT).as_posix()+'/';newprefix=A.relative_to(ROOT).as_posix()+'/'
    required={};copies=[]
    for p,h in old['frozen_inputs'].items():
        assert sha(ROOT/p)==h,p
        if not p.startswith(oldprefix):required[p]=h;continue
        rel=p[len(oldprefix):];dest=A/rel;assert not dest.exists();dest.parent.mkdir(parents=True,exist_ok=True)
        data=(ROOT/p).read_bytes()
        if dest.suffix in ('.py','.json','.md','.sh','.bash'):
            text=data.decode('utf-8').replace(OLD.name,A.name)
            if rel=='steering_runtime_audit.py':
                assert text.count("expected={'local_pursuit':LIMIT")==1
                text=text.replace("expected={'local_pursuit':LIMIT","expected={'g1_g2_handoff_trial':LIMIT")
            data=text.encode('utf-8')
        dest.write_bytes(data);required[newprefix+rel]=sha(dest)
        copies.append(dict(path=rel,previous_sha256=h,after_sha256=sha(dest),path_relocated=True,logic_change=rel=='steering_runtime_audit.py'))
    # 실제 호출/상한값은 첫 시도의 원 Node 생성 기록으로 좁게 확인한다.
    actual=read(OLD/'steering_runtime_receipt.json');nodes={v['executable']:v['max_steering_angle_rad'] for v in actual['actual_node_parameters']}
    assert nodes['g1_g2_handoff_trial']==nodes['lidar_safety']==.28475436568260193
    assert all(v for k,v in actual['checks'].items() if k!='local_pursuit')
    expected=read(A/'expected_header.json');assert expected==read(OLD/'expected_header.json')
    cfg=read(A/'config.json');restore=json.loads(json.dumps(cfg).replace(A.name,OLD.name));assert restore==read(OLD/'config.json')
    assert sha(A/'field_world.sdf')==sha(OLD/'field_world.sdf')
    assert (A/'arguments.json').read_text().replace(A.name,OLD.name)==(OLD/'arguments.json').read_text()
    receipt=dict(passed=True,attempt1_report_sha256=sha(OLD/'run1/report.json'),attempt1_source_archive_sha256=sha(OLD/'sources.zip'),
        attempt1_started_children=0,attempt1_pid0=True,required_sources_unchanged=True,condition_change=False,
        logic_diff="steering_runtime_audit.py expected controller executable: local_pursuit -> g1_g2_handoff_trial",
        numerical_checks_unchanged=True,production_changed=False,files=copies,one_additional_launch_authorized=True,repeat_on_failure=False)
    write(A/'attempt_preparation.json',receipt)
    for p in (A/'attempt_preparation.json',Path(__file__).resolve()):required[p.relative_to(ROOT).as_posix()]=sha(p)
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    old_observed=old['observed_inputs']
    with zipfile.ZipFile(OLD/'sources.zip') as source,zipfile.ZipFile(A/'sources.zip','x',zipfile.ZIP_DEFLATED) as target:
        for p,h in {**required,**old_observed}.items():
            if p.endswith('.zip'):continue
            data=(ROOT/p).read_bytes() if p.startswith(newprefix) else source.read(p)
            assert hashlib.sha256(data).hexdigest()==h,p;target.writestr(p,data)
    prepared={**old,'args':read(A/'arguments.json'),'frozen_inputs':required,'source_archive_sha256':sha(A/'sources.zip'),
        'preparation_reused_from':OLD.relative_to(ROOT).as_posix(),'previous_launch_without_started_children':True}
    write(A/'prepared.json',prepared)
    print(json.dumps(dict(required=len(required),observed=len(old_observed),prepared_sha256=sha(A/'prepared.json'),archive_sha256=sha(A/'sources.zip'))))
if __name__=='__main__':main()
