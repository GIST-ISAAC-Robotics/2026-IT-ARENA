"""후속 자원 배정 이후에만 사용하는 독립 ARM 계산 recipe. 현재 준비 중에는 호출하지 않는다."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import traceback

from environment_reference import other_work, safe_environment_snapshot

ROOT=Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert os.uname().machine=='aarch64' and ROOT.parent==Path.home() and ROOT.name.startswith('it-arena-r462-')
    manifest_path=ROOT/'source_manifest.json'
    manifest=json.loads(manifest_path.read_bytes())
    review=json.loads((ROOT/'execution_review.json').read_bytes())
    assert review['root_reviewed'] is True and review['jetson_exclusive_resource_assigned'] is True
    assert review['manifest_sha256']==sha(manifest_path) and review['planned_total_computations']==13
    assert review['repeat_on_failure'] is False
    assert not (ROOT/'execution_receipt.json').exists() and not (ROOT/'results').exists()
    interference=other_work(ROOT)
    assert not any(interference.values()),interference
    def changed():
        return [p for p,h in manifest['files'].items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
    assert not changed()
    native=manifest['native_reuse']
    source=Path(native['source'])
    assert sha(source)==native['sha256']
    destination=ROOT/'src/arena_autonomy/arena_autonomy'/source.name
    assert not destination.exists()
    shutil.copyfile(source,destination)
    assert sha(destination)==native['sha256']
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=os.pathsep.join(
        [str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT)]))
    before=safe_environment_snapshot()
    steps=[]
    error=None
    try:
        for mode in ('simulation','estimated','summarize'):
            command=[sys.executable,'-B',str(ROOT/'run_contract.py'),mode]
            with (ROOT/(mode+'.log')).open('x') as log:
                child=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                row=dict(mode=mode,command=command,pid=child.pid,timeout_s=60,forced=False)
                steps.append(row)
                try:
                    row['returncode']=child.wait(timeout=60)
                except subprocess.TimeoutExpired:
                    row['forced']=True
                    os.killpg(child.pid,signal.SIGINT)
                    try:row['returncode']=child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGKILL)
                        row['returncode']=child.wait(timeout=5)
                try:os.killpg(child.pid,0)
                except ProcessLookupError:row['residual_group']=False
                else:
                    row['residual_group']=True
                    os.killpg(child.pid,signal.SIGKILL)
                assert row['returncode']==0 and not row['forced'] and not row['residual_group'],row
    except BaseException as exc:
        error=repr(exc)
        (ROOT/'traceback.txt').write_text(traceback.format_exc())
    finally:
        receipt=dict(computation_collection_completed=error is None,steps=steps,error=error,
            source_changes=changed(),native_sha256=sha(destination),native_reused=True,new_build=False,
            before=before,after=safe_environment_snapshot(),remaining=other_work(ROOT),
            hardware_activation=False,ports_opened=False,physical_measurement=False,
            scope='saved simulation numeric input contract comparison only; no ROS or real-time guarantee')
        (ROOT/'execution_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return 0 if error is None and not receipt['source_changes'] and not any(receipt['remaining'].values()) else 1


if __name__=='__main__':
    raise SystemExit(main())
