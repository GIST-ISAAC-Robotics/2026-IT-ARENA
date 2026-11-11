"""81T·r75 유지, r77 뒤쪽 구조 기억 중복 판정 보완의 제한 native 단독1회. 등록/결과를 덮어쓰지 않는다."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from run_new_course_solo import preset_args

output=HERE/'solo_run1';registration=HERE/'registered.json'
if output.exists() or registration.exists():raise SystemExit('trial already registered; preserve evidence')
args=preset_args('new-course-solo',output)
args.remove('--video')
args[args.index('--local-sensor-config')+1]=str(ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml')
args.extend(['--lidar-native-sampling-profile',str(ROOT/'config/tests/c1_standard10_native_angles_v1.json')])
files=[p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.xml','.xacro','.sdf','.cpp','.hpp')]
files.extend(p for p in (ROOT/'scripts').glob('*.py'))
files += [ROOT/'config/tests/c1_standard10_native_angles_v1.json',Path(__file__)]
digests={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(files))}
registration.write_text(json.dumps(dict(args=args,input_sha256=digests,
    changes=['r77 explained rear structure points already in hidden memory use that existing memory sweep, not duplicate moving-loss latch','r75 bounded longitudinal surface correspondence retained','r73 81T/17T times same 2.5 reduction, 28 counts per motor revolution, 65mm wheel retained','r74 forward-prefix policy present but requires junction continuity; not a paired deterministic A/B'],
    time_model='simulated period/count; NOT verified hardware acquisition timestamps',
    sensor_baseline='current measured-rate/scatter IMU, zero mean; existing B0390/JR60P20',
    scope='r75 failure followup after sensor-only memory diagnosis and regression; one solo closed loop with STOP; no torque/friction/controller gain or gap threshold change; not hardware proof'),indent=2)+'\n',encoding='utf-8')
code=subprocess.call([sys.executable,'-B',*args],cwd=ROOT)
changed=[name for name,digest in digests.items() if not (ROOT/name).exists()
         or hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest]
(HERE/'run_integrity.json').write_text(json.dumps(dict(process_exit=code,changed_inputs=changed,
    input_count=len(digests),unchanged=not changed),indent=2)+'\n',encoding='utf-8')
raise SystemExit(code or int(bool(changed)))
