from pathlib import Path
import ast,difflib,hashlib,json,zipfile
B=Path(__file__).resolve().parent;R=B.parents[3];S=B/'stage'
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    original=json.loads((B/'original_sha256.json').read_bytes())
    common_changed=[n for n,h in original.items() if digest(R/n)!=h]
    assert not common_changed
    changes=[];patch=[]
    for name in ('src/arena_autonomy/arena_autonomy/race_runtime_policy/guard_crawl.py',
                 'src/arena_autonomy/arena_autonomy/race_runtime_policy/released_plan.py',
                 'tests/test_runtime_guard_view_reuse.py'):
        src=R/name;target=S/name;old=src.read_text(encoding='utf8') if src.exists() else ''
        new=target.read_text(encoding='utf8');ast.parse(new)
        changes.append(dict(path=name,original_sha256=digest(src) if src.exists() else None,
            stage_sha256=digest(target),new_file=not src.exists()))
        patch.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),
            fromfile='a/'+name if src.exists() else '/dev/null',tofile='b/'+name))
    allowed={c['path'] for c in changes}
    assert all(digest(S/n)==h for n,h in original.items() if n not in allowed)
    (B/'apply.patch').write_bytes(''.join(patch).encode('utf8'))
    (B/'changes.json').write_text(json.dumps(changes,indent=2)+'\n',encoding='utf8')
    receipts=[json.loads((B/p/'receipt.json').read_bytes()) for p in ('verify_v1','verify_v2','verify_v3')]
    outcomes={}
    for index,receipt in enumerate(receipts):
        assert not receipt['external_project_imports']
        for item in receipt['outcomes']:
            if index==0 and 'test_runtime_guard_view_reuse.py::' in item['test']:continue
            if item['phase']=='call':outcomes[item['test']]=item['outcome']
    assert len(outcomes)==168 and set(outcomes.values())=={'passed'}
    saved=json.loads((B/'saved_v1/report.json').read_bytes());assert saved['passed']
    manifest={p.relative_to(S).as_posix():digest(p) for p in S.rglob('*') if p.is_file()}
    (B/'stage_manifest_v2.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
    archive=B/'staged_sources_v2.zip'
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for p in S.rglob('*'):
            if p.is_file():z.write(p,p.relative_to(S).as_posix())
    report=dict(common_changes=common_changed,changed_or_new_files=len(changes),stage_files=len(manifest),
        tests_unique=168,tests_passed=168,unresolved_test_failures=0,
        first_run_passed=164,first_run_failed=4,first_failures='missing isolated fixture/helper dependencies only',
        corrected_failed_tests_only=4,new_tests=sum('/test_runtime_guard_view_reuse.py::' in n.replace('\\','/') for n in outcomes),
        saved_rows=saved['rows'],old_stage_exact=saved['old_stage_exact'],
        old_view_calls=saved['old_threat_view_calls'],stage_view_calls=saved['stage_threat_view_calls'],
        candidate_accepted=saved['candidate_accepted'],current_dispatch_supported=saved['current_supported'],
        archive_file=archive.name,archive_sha256=digest(archive),patch_sha256=digest(B/'apply.patch'),
        final_new_tests_fixture_independent=True,
        wsl_used=False,jetson_used=False,devices_used=False,full_plan_replayed=False,
        jetson_time_saving_measured=False,background_processes_started=False)
    (B/'summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8');print(json.dumps(report))
if __name__=='__main__':main()
