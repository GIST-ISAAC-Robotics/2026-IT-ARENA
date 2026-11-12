from pathlib import Path
import subprocess,os,signal,json,hashlib,tarfile
root=Path('/home/q/it_arena_c1_preview_r204_20261112')
a=root/'artifacts/validation/2026-11-12/c1_nonreturn_live_r204_v1'
device=Path('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_362acd64b231f11190a78f5f30d20014-if00-port0').resolve(strict=True)
assert str(device)=='/dev/ttyUSB0'
assert subprocess.run(['fuser',str(device)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==1
acl=subprocess.check_output(['getfacl','--absolute-names',str(device)])
backup=root/'device_acl.before';backup.write_bytes(acl)
result={};process=None
try:
 subprocess.run(['setfacl','-m','u:q:rw',str(device)],check=True)
 command="source /opt/ros/humble/setup.bash; cd /home/q/it_arena_c1_preview_r204_20261112; export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=216 PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:${PYTHONPATH:-}; python3 -B scripts/bridge_c1_native_preview.py --executable ./stream_c1_native_preview --port /dev/ttyUSB0 --seconds 20 --output artifacts/validation/2026-11-12/c1_nonreturn_live_r204_v1/capture"
 with (root/'run.log').open('x') as log:
  process=subprocess.Popen(['runuser','-u','q','--','bash','-lc',command],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  try:result['run_exit']=process.wait(timeout=70)
  except subprocess.TimeoutExpired:
   result['timeout']=True;os.killpg(process.pid,signal.SIGINT)
   try:process.wait(8)
   except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
finally:
 subprocess.run(['setfacl','--restore='+str(backup)],check=True)
 result['acl_restored']=subprocess.check_output(['getfacl','--absolute-names',str(device)])==acl
 result['device_owner_remaining']=subprocess.run(['fuser',str(device)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode!=1
 manifest=json.loads((a/'source_manifest.json').read_text())
 result['source_unchanged']=all(hashlib.sha256((root/p).read_bytes()).hexdigest()==h for p,h in manifest.items())
 (a/'run_result.json').write_text(json.dumps(result,indent=2))
 with tarfile.open('/home/q/r204_results_private.tar.gz','w:gz') as t:
  for p in [a/'capture',a/'run_result.json',root/'run.log']: 
   if p.exists():t.add(p,arcname=p.relative_to(root).as_posix())
 print(json.dumps(result))
 if (a/'capture/summary.json').exists():print((a/'capture/summary.json').read_text())
assert result.get('run_exit')==0 and result['acl_restored'] and not result['device_owner_remaining'] and result['source_unchanged'] and not result.get('timeout')
