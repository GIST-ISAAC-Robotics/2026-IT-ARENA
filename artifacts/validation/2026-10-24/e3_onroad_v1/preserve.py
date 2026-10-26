"""두 실행의 원래 해시와 일치하는 현재 파일만 복사. 원시 결과는 덮어쓰지 않는다."""
import hashlib
import json
from pathlib import Path
import shutil

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]


if __name__ == '__main__':
    snapshots = OUT/'source_snapshot'
    snapshots.mkdir(exist_ok=False)
    wanted = {}
    reports = {}
    for mode in ('g1','r0plus'):
        p = OUT/mode/'report.json'
        r = json.loads(p.read_text())
        reports[mode] = hashlib.sha256(p.read_bytes()).hexdigest()
        for name, digest in r['source_sha256'].items():
            if name in wanted and wanted[name] != digest:
                raise ValueError('Run sources differed: '+name)
            wanted[name] = digest
    for name, digest in wanted.items():
        src = ROOT/name
        if hashlib.sha256(src.read_bytes()).hexdigest() != digest:
            raise ValueError('Changed source: '+name)
        dst = snapshots/name
        dst.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(src,dst)
        if hashlib.sha256(dst.read_bytes()).hexdigest() != digest:
            raise ValueError('Copy mismatch: '+name)
    result = {'source_files':wanted,'reports':reports,'scope':'exact source copy verified against both original run manifests'}
    with (OUT/'preservation.json').open('x') as f:
        json.dump(result,f,indent=2)
    print(json.dumps({'files':len(wanted),'both_run_sources_preserved':True}))
