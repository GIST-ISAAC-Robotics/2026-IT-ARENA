"""새 Jetson 복사본의 bounded 빌드·입력 결합·회수. 외부 SSH는 실행 담당이 수행한다."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import zipfile


def main():
    p=argparse.ArgumentParser();p.add_argument('--archive',type=Path,required=True)
    p.add_argument('--sha256',required=True);p.add_argument('--root',type=Path,required=True)
    args=p.parse_args();root=args.root.resolve()
    assert sys.platform=='linux' and os.uname().machine=='aarch64'
    assert root.parent==Path.home() and root.name.startswith('it-arena-r404-') and not root.exists()
    assert hashlib.sha256(args.archive.read_bytes()).hexdigest()==args.sha256
    with zipfile.ZipFile(args.archive) as archive:
        assert all(not Path(n).is_absolute() and '..' not in Path(n).parts and '\\' not in n for n in archive.namelist())
        root.mkdir();archive.extractall(root)
    work=root/'artifacts/validation/2026-11-13/runtime_live_r404_v1'
    source=json.loads((root/'source_manifest.json').read_bytes())['files']
    def changed():
        return [n for n,h in source.items() if not (root/n).is_file() or hashlib.sha256((root/n).read_bytes()).hexdigest()!=h]
    assert not changed()
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',ROS_DOMAIN_ID='220',ROS_LOCALHOST_ONLY='1')
    env['PYTHONPATH']=':'.join([str(root/'src/arena_autonomy'),str(root/'src/arena_vehicle_interface'),str(root/'scripts'),os.environ.get('PYTHONPATH','')])
    steps=[];error=None;began=time.monotonic()
    def execute(name,command,cwd,seconds):
        with (work/(name+'.log')).open('x') as log:
            child=subprocess.Popen(command,cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            row=dict(name=name,pid=child.pid,argv=command,timeout_s=seconds,forced=False);steps.append(row)
            try:row['returncode']=child.wait(timeout=seconds)
            except subprocess.TimeoutExpired:
                row['forced']=True;child.send_signal(signal.SIGINT)
                try:row['returncode']=child.wait(timeout=50.)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);row['returncode']=child.wait(timeout=5.)
                raise
            if row['returncode']!=0:raise RuntimeError((name,row['returncode']))
    try:
        execute('native_build',[sys.executable,'-B','setup.py','build_ext','--inplace'],root/'src/arena_autonomy',120)
        execute('imports',[sys.executable,'-B','-c',
            'import serial,rclpy,gi; from ackermann_msgs.msg import AckermannDriveStamped; from arena_autonomy import numeric_kernels; numeric_kernels.configure("native_v1"); print("runtime imports ready")'],root,15)
        execute('live',[sys.executable,'-B',str(work/'run_live.py'),'--output',str(work/'actual')],root,85)
    except BaseException as exc:error=type(exc).__name__+': '+str(exc)
    finally:
        result=dict(error=error,steps=steps,elapsed_s=time.monotonic()-began,source_changes=changed(),
            source_files=len(source),archive_sha256=args.sha256,physical_actuation=False,firmware_written=False)
        (work/'execution.json').write_text(json.dumps(result,indent=2)+'\n')
        target=args.archive.with_name(args.archive.stem+'-results.zip')
        with zipfile.ZipFile(target,'x',zipfile.ZIP_DEFLATED) as archive:
            for path in work.rglob('*'):
                if path.is_file() and (path.is_relative_to(work/'actual') or path.suffix=='.log' or path.name=='execution.json'):
                    archive.write(path,path.relative_to(work))
        print(json.dumps(result))
    if error or result['source_changes']:raise SystemExit(1)


if __name__=='__main__':main()
