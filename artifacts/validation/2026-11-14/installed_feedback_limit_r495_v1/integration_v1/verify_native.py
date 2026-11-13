from pathlib import Path
import hashlib,json,subprocess,tempfile,time
A=Path(__file__).resolve().parent; ROOT=A.parents[4]
source=A.parent/'receiver_profile_contract.cpp'
commands=[]; start=time.monotonic()
with tempfile.TemporaryDirectory(prefix='arena-r495-') as td:
 target=Path(td)/'receiver_profile_contract'
 command=['g++','-std=c++17','-O2','-I'+str(ROOT/'src/arena_vehicle_interface/firmware_runtime_drive/main'),'-I'+str(ROOT/'src/arena_vehicle_interface/portable'),str(source),'-o',str(target)]
 for name,cmd in [('compile',command),('run',[str(target)])]:
  p=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=30)
  (A/(name+'.log')).write_text(p.stdout,encoding='utf-8'); commands.append(dict(name=name,exit_code=p.returncode,command=cmd))
  if p.returncode:break
record=dict(commands=commands,passed=len(commands)==2 and all(c['exit_code']==0 for c in commands),elapsed_s=time.monotonic()-start,contract_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),device_io=False,firmware_build=False,installed_values_selected=False,cpp_fixture_correction='explicit synthetic servo.command_limit=.2; default zero is unconfigured')
(A/'native_receipt.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
print(json.dumps(record));raise SystemExit(0 if record['passed'] else 1)
