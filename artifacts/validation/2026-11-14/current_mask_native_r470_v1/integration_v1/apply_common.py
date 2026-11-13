"""승인된 r470 구현5파일과 회귀1파일만 before hash 확인 뒤 공통에 이관한다."""
import hashlib,json,shutil
from pathlib import Path

A=Path(__file__).resolve().parent
R470=A.parent
ROOT=A.parents[4]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    assert not (A/'source_receipt.json').exists()
    rows=json.loads((R470/'changes.json').read_bytes())+[
        dict(path='tests/test_current_mask_native.py',before=None,
            after='ecce49902cdd3dcad22e46578d79c0a6fcd0bde6898f3d872530cb805aa7e341')]
    for row in rows:
        destination=ROOT/row['path'];source=R470/'stage'/row['path']
        assert (sha(destination) if destination.exists() else None)==row['before'],row['path']
        assert sha(source)==row['after'],row['path']
    before=A/'before';before.mkdir(exist_ok=False)
    for row in rows:
        source=ROOT/row['path']
        if source.exists():
            saved=before/row['path'];saved.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(source,saved);assert sha(saved)==row['before']
    binaries=[]
    for source in (ROOT/'src/arena_autonomy/arena_autonomy').glob('_numeric_kernels*.so'):
        saved=before/source.relative_to(ROOT);saved.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(source,saved)
        binaries.append(dict(path=source.relative_to(ROOT).as_posix(),sha256=sha(source)))
    for row in rows:
        destination=ROOT/row['path'];destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(R470/'stage'/row['path'],destination)
        assert sha(destination)==row['after']
    receipt=dict(common_integrated=True,files=rows,source_file_count=6,
        previous_binaries=binaries,source_before_hash_match=True,previous_sources_preserved=True,
        wsl_build_pending=True,jetson_used=False,commit_or_push=False)
    (A/'source_receipt.json').write_text(json.dumps(receipt,indent=2))
    print(json.dumps(receipt))

if __name__=='__main__':main()
