"""관측 IMU 평균을 편향으로 가정한 코너 갱신 제한 단독1회. 등록/결과를 덮어쓰지 않는다."""
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
args[args.index('--local-sensor-config')+1]=str(HERE/'sensor.yaml')
args.extend(['--motion-test-profile',str(HERE/'motion.json')])
files=[p for p in (ROOT/'src').rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.xml','.xacro','.sdf','.cpp','.hpp')]
files.extend(p for p in (ROOT/'scripts').glob('*.py'))
files += [HERE/'sensor.yaml',HERE/'motion.json',Path(__file__)]
digests={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(files))}
registration.write_text(json.dumps(dict(args=args,input_sha256=digests,
    changes=['r71 current observed solo corner reuses existing shared arc agreement','r72 native CPU optimization present but this uniform legacy run uses original geometry path'],
    time_model='legacy sequential simulation; NOT hardware acquisition timestamps',
    sensor_baseline='same observed-sensitivity IMU rate/scatter and assumed bias -0.015615217079382257 rad/s; uniform C1/B0390/JR60P20',
    scope='one solo closed loop, not sensor calibration or deterministic paired A/B'),indent=2)+'\n',encoding='utf-8')
code=subprocess.call([sys.executable,'-B',*args],cwd=ROOT)
changed=[name for name,digest in digests.items() if not (ROOT/name).exists()
         or hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest]
(HERE/'run_integrity.json').write_text(json.dumps(dict(process_exit=code,changed_inputs=changed,
    input_count=len(digests),unchanged=not changed),indent=2)+'\n',encoding='utf-8')
raise SystemExit(code or int(bool(changed)))
