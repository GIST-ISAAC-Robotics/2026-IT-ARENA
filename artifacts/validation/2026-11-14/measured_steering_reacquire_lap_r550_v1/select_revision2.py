"""root 최종 revision2 인계를 명시 반영한다. 시험/주행을 실행하지 않는다."""
import hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];BASE=A.parent/'measured_steering_lap_r544_attempt2_v1'
read=lambda p:json.loads(Path(p).read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2);f.write('\n')
def main():
    assert not any((A/p).exists() for p in ('prepared.json','sources.zip','registered.json','run1','revision1_source_selection.json'))
    selection=read(A/'source_selection.json');write(A/'revision1_source_selection.json',selection)
    expected={
      'src/arena_autonomy/arena_autonomy/current_wall_reacquire.py':'598e1d70a61b5fb9757eacba6fd1f4c8bff42d375df2accb7ed9802171b75d1b',
      'src/arena_autonomy/arena_autonomy/local_path.py':'9ee7d0e3761d96084b3c71c4ef3781c50484f7ea9e2c2f14a93cc3afe0c816ad'}
    for p,h in selection['required_sources'].items():assert sha(ROOT/p)==expected.get(p,h),p
    baseline={**read(BASE/'prepared.json')['frozen_inputs'],**read(BASE/'run1/report.json')['source_sha256']}
    selection['required_sources'].update(expected)
    selection['source_delta']=[dict(path=p,before_sha256=baseline[p],after_sha256=h) for p,h in selection['required_sources'].items() if p in baseline and baseline[p]!=h]
    regression=A.parent/'current_short_path_r549_v1/revision2';proofs={}
    for name,count in [('regression.xml',42),('final_helper.xml',15)]:
        path=regression/name;rows=ET.parse(path).getroot().findall('.//testcase')
        assert len(rows)==count and all(row.find('failure') is None and row.find('error') is None and row.find('skipped') is None for row in rows)
        proofs[path.relative_to(ROOT).as_posix()]=dict(sha256=sha(path),passed=count)
    selection.update(freeze_authorized=True,wsl_exclusive_owner=True,drive_authorized=True,final_production_source_selection_pending=False,
        native_rebuild_authorized=True,status='root final revision2 handover; one solo run only; no repeat on harness or driving failure',
        revision='r549 revision2 preferred wall short observation; opposite-wall fallback removed',validation=proofs)
    (A/'source_selection.json').write_text(json.dumps(selection,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    write(A/'revision2_selection_receipt.json',dict(passed=True,source_selection_sha256=sha(A/'source_selection.json'),source_delta=selection['source_delta'],
        validation=proofs,world_sha256=sha(A/'field_world.sdf'),header_sha256=sha(A/'expected_header.json'),status_detail='full',
        command_limit_rad=.28475436568260193,joint_limit_rad=.30666234131856146,WSL_started=False,actual_runs=0))
    print(json.dumps(dict(selected=True,source_selection_sha256=sha(A/'source_selection.json'),source_delta=selection['source_delta'])))
if __name__=='__main__':main()
