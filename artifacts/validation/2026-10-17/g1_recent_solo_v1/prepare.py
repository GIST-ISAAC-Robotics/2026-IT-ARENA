"""최신 통과한 회귀 소스와 동일한지 대조 후 단독 1회 입력을 보존한다."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).resolve().parent

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    assert not (OUT/'pre_run_manifest.json').exists(), '기존 실행을 덮어쓰지 않습니다.'
    baseline=ROOT/'artifacts/validation/2026-10-17/g1_recent_regression_v2'
    assert '2708 passed' in (baseline/'pytest.log').read_text()
    assert (baseline/'source_hashes_before.sha256').read_bytes()==(baseline/'source_hashes_after.sha256').read_bytes()
    paths=[];changed=[]
    for line in (baseline/'source_hashes_after.sha256').read_text().splitlines():
        expected,name=line.split('  ',1)
        path=ROOT/name
        if digest(path)!=expected:changed.append(name)
        paths.append(path)
    ros=json.loads((ROOT/'artifacts/validation/2026-10-17/g1_recent_observation_ros_v3/run/validator_output/report.json').read_text())
    checks=dict(source_unchanged=not changed,ros_passed=ros['passed'],
                ros_43_cases=len(ros['cases'])==43,
                no_cleanup_errors=not ros['cleanup_errors'],
                no_remaining_sessions=not ros['remaining_session_members'])
    report=dict(passed=all(checks.values()),checks=checks,changed=changed,
                validation_scope='bounded_offline_run_not_realtime_deadline')
    (OUT/'preflight_integrity.json').write_text(json.dumps(report,indent=2)+'\n')
    assert report['passed'],report
    paths.extend(OUT/name for name in ('README.md','prepare.py','run.sh'))
    paths.extend(ROOT/name for name in ('.gitignore','.gitattributes'))
    paths=sorted(set(paths))
    manifest=[dict(path=p.relative_to(ROOT).as_posix(),sha256=digest(p)) for p in paths]
    (OUT/'pre_run_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    with zipfile.ZipFile(OUT/'pre_run_source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in paths:archive.write(p,p.relative_to(ROOT).as_posix())
    print(json.dumps(dict(passed=True,snapshot_files=len(paths))))

if __name__=='__main__':main()
