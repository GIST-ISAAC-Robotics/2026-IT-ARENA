"""표준 라이브러리로 AST/문자열/파일 해시만 검사한다. 대상 코드를 import하지 않는다."""
import ast
import hashlib
import json
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
OLD=A.parent/'final_geometry_crawl_drive_r353_v1'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    origin=json.loads((A/'copy_origin.json').read_bytes())
    unchanged=[r['path'] for r in origin['records'] if sha(OLD/r['path'])==r['source_sha256']]
    assert len(unchanged)==len(origin['records'])
    parsed={p.relative_to(A).as_posix():ast.parse(p.read_text(encoding='utf-8'),filename=str(p))
            for p in A.rglob('*.py')}
    exact=[]
    for name in ['launch_prefix.py','profile.json','stop_owned.py','course.launch.py',
                 'launch/demo.launch.py','launch/local_pursuit.launch.py','launch/simulation.launch.py','actors.json']:
        expected=(OLD/name).read_text(encoding='utf-8').replace('final_geometry_crawl_drive_r353_v1',
            'released_crawl_drive_r364_v1').replace('r353','r364')
        assert (A/name).read_text(encoding='utf-8')==expected,name
        exact.append(name)
    old=json.loads((OLD/'config.json').read_bytes());new=json.loads((A/'config.json').read_bytes())
    old.pop('scope');new.pop('scope');old['launch'].pop('entry_g1');new['launch'].pop('entry_g1')
    assert old==new,'non-description configuration changed'
    worker=(A/'low_solo_worker.py').read_text(encoding='utf-8')
    assert worker.index('boundary.install()')<worker.index('with policy.enabled():')<worker.index('prior.ORIGINAL(stage, *args)')
    assert "A.parent/'released_crawl_r361_v1/candidate.py'" in worker
    assert "A.parent/'solo_crawl_retry_r352_v1/candidate.py'" in worker
    assert "route_proof.rebase = policy.rebase" in worker
    test=(A/'test_node.py').read_text(encoding='utf-8')
    assert "A = Path(__file__).resolve().parent" in test and "A/'node_entry.py'" in test
    assert 'released_plan_crawl_policy' in test and 'selector_patch_installed' in test
    node=(A/'node_entry.py').read_text(encoding='utf-8')
    assert 'released_plan_crawl_policy' in node and 'stage=\'plan_only\'' in node
    runner=(A/'run.py').read_text(encoding='utf-8')
    assert "(ROOT/'tests').rglob('*.py')" in runner
    assert "(A.parent/'released_crawl_r361_v1').glob('*.py')" in runner
    assert "'--vehicle-config', str(A.parent/'final_vehicle_geometry_r350_v1/vehicle.final.yaml')" in runner
    assert not any((A/name).exists() for name in ('run1','registered.json','sources.zip','worker_policy_receipts'))
    prepared={p.relative_to(A).as_posix():sha(p) for p in sorted(A.rglob('*'))
              if p.is_file() and p.name!='static_audit.json'}
    pending=[ROOT/'src/arena_autonomy/arena_autonomy'/name for name in
             ('runtime_dual_process.py','runtime_dual_state.py','runtime_dual_delivery.py')]
    report=dict(ast_files=len(parsed),ast_paths=sorted(parsed),r353_sources_unchanged=unchanged,
        renamed_only_exact=exact,configuration_equal_except_scope_and_entry_description=True,
        r361_candidate_sha256=sha(A.parent/'released_crawl_r361_v1/candidate.py'),
        prepared_sha256=prepared,r362_preparation_snapshot_advisory={p.relative_to(ROOT).as_posix():sha(p) for p in pending},
        execution_source_freeze_complete=False,node_test_run=False,build_run=False,wsl_run=False,
        drive_run=False,scope='AST and source preparation only; parent must review/freeze before execution')
    (A/'static_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(ast_files=len(parsed),r353_sources_unchanged=len(unchanged),
        renamed_only_exact=len(exact),node_test_run=False,drive_run=False)))


if __name__=='__main__':main()
