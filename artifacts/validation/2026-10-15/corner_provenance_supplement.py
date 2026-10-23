"""누락된 wrapper 원문을 사후 보존한다. 실행 전 해시가 있었다고 소급하지 않는다."""
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import zipfile

BASE=Path(__file__).resolve().parent


def main():
    output=BASE/'corner_provenance_supplement.json'
    if output.exists():raise FileExistsError(output)
    rows=[]
    names=('g1_corner_evidence_v2','g1_corner_geometry_v2','g1_corner_distance_v2')
    with zipfile.ZipFile(BASE/'corner_wrapper_supplement.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name in names:
            folder=BASE/name
            files=[folder/'probe.py',folder/'run.sh']
            row=dict(folder=name,files={},boundary='post_execution_capture_not_original_pre_execution_attestation')
            for path in files:
                if not path.exists():continue
                rel=str(path.relative_to(BASE))
                row['files'][rel]=hashlib.sha256(path.read_bytes()).hexdigest()
                z.write(path,rel)
            rows.append(row)
        z.write(__file__,Path(__file__).name)
    report=dict(captured_at_utc=datetime.now(timezone.utc).isoformat(),
        scope='append_only_wrapper_provenance_supplement',original_artifacts_overwritten=False,
        explanation='Imported v1 main used its own __file__ for snapshots. Actual v2 wrappers are preserved here after execution. This does not certify an originally absent pre-run wrapper hash.',
        rows=rows,archive_sha256=hashlib.sha256((BASE/'corner_wrapper_supplement.zip').read_bytes()).hexdigest())
    output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps(dict(folders=len(rows),original_artifacts_overwritten=False)))


if __name__=='__main__':main()
