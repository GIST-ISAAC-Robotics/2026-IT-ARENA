"""선정한 원입력 세 행만 fresh LocalFollow로 계산. 기존 상태 prefix 재현 아님."""
from pathlib import Path
import hashlib
import json
import os
import sys
import time
import traceback

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
REPO=ROOT.parents[3]
for package in ('arena_autonomy','arena_vehicle_interface'):
    sys.path.insert(0,str(HERE/'frozen_source/src'/package))
import arena_autonomy
native=REPO/'src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-312-x86_64-linux-gnu.so'
native_sha=hashlib.sha256(native.read_bytes()).hexdigest()
assert native_sha=='1bbe89cdcff6662fda699847756ff9c776e1b70c0d406ec97df469666dc3b16c',native_sha
arena_autonomy.__path__.append(str(native.parent))
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import LocalFollow,FollowConfig
import arena_autonomy.local_follow as follow_module

rows=[json.loads(line) for line in (ROOT/'run1/follow_decisions.jsonl').read_text(encoding='utf8').splitlines()]
header=next(r['data']['header'] for r in rows if r['kind']=='runtime_evaluator_header')
results=[]
(HERE/'fresh_baseline.pid').write_text(str(os.getpid())+'\n',encoding='utf8')
print('r478 fresh baseline started pid='+str(os.getpid()),flush=True)
for seq in (563,650,740):
    row=next(r for r in rows if r['kind']=='call' and r['seq']==seq)
    d,scan,history=restore_call(row['data']['input'],MotionConfig(**header['motion_config']))
    scene=motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],
        epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    calls=[]
    original=follow_module.swept_limit
    def observe(*args,**kwargs):
        value=original(*args,**kwargs)
        calls.append({'steering_rad':float(args[1]),'clearance_m':float(value),'horizon_m':kwargs.get('horizon')})
        return value
    follow_module.swept_limit=observe
    began=time.monotonic()
    try:
        output=LocalFollow(FollowConfig(**header['follow_config'])).command(scene,history,
            d['speed_mps'],d['side'],d['previous_steering_rad'],d['dt_s'],raw_scan=scan)
        result={'seq':seq,'reference_s':d['reference_s'],'actual_speed_mps':d['speed_mps'],
            'actual_previous_steering_rad':d['previous_steering_rad'],'output':freeze(output),
            'baseline_swept_calls':calls,'elapsed_s':time.monotonic()-began}
        print(json.dumps({'seq':seq,'speed':output[0],'steering':output[1],
            'reason':output[2].get('reason'),'swept':calls}),flush=True)
    except Exception as error:
        result={'seq':seq,'error':type(error).__name__,'message':str(error),'traceback':traceback.format_exc()}
        print(json.dumps(result),flush=True)
    finally:
        follow_module.swept_limit=original
    results.append(result)
receipt={'scope':'각 행의 원 scan/history/속도/직전각/dt 그대로; 새 LocalFollow를 행마다 생성한 isolated probe. MovingPass 또는 기존 LocalFollow 상태 prefix 재현 아님.',
    'native_sha256':native_sha,'pid':os.getpid(),'rows':results,
    'source_archive_sha256':hashlib.sha256((ROOT/'sources.zip').read_bytes()).hexdigest(),
    'input_trace_sha256':hashlib.sha256((ROOT/'run1/follow_decisions.jsonl').read_bytes()).hexdigest()}
(HERE/'fresh_baseline_wsl.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
sys.exit(int(any('error' in r for r in results)))
