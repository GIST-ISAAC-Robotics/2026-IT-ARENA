"""첫 ROS 실패의 동일 합성 RGB 두 장만 구 API detector 좌표로 진단한다."""
import json,os
from pathlib import Path
import cv2
import numpy as np
from arena_autonomy.core import marker_observations

artifact=Path('/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/vehicle_marker_r466_v1')
work='/home/jinhyeong/it-arena-r466-ros-a'
dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
draw=getattr(cv2.aruco,'generateImageMarker',None) or cv2.aruco.drawMarker
rows=[]
for name,size,x,y in [('front',80,640,440),('side',48,100,420)]:
    rgb=np.full((720,1280,3),255,np.uint8)
    rgb[y-size//2:y+size//2,x-size//2:x+size//2]=draw(dictionary,1,size)[:,:,None]
    observations=marker_observations(rgb,.02)
    rows.append(dict(name=name,observations=[dict(id=i,corners=np.asarray(q).tolist(),
        center=np.asarray(q).mean(axis=0).tolist(),mean_edge=float(np.linalg.norm(np.roll(q,-1,axis=0)-q,axis=1).mean()))
        for i,q in observations]))
original=json.loads((artifact/'execution_v1/run1/result.json').read_bytes())
owned=[json.loads((artifact/'execution_v1/run1/pid.json').read_bytes())['pid'],*original['child_processes']]
alive=[pid for pid in owned if Path('/proc',str(pid)).exists()]
other=[]
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit() or int(entry.name) in (os.getpid(),os.getppid()):continue
    try:cmd=(entry/'cmdline').read_bytes().decode(errors='replace').replace('\0',' ')
    except (OSError,ProcessLookupError):continue
    if work in cmd:other.append(dict(pid=int(entry.name),command=cmd))
result=dict(opencv=cv2.__version__,observations=rows,owned_pids_from_run=owned,owned_alive=alive,
    other_snapshot_processes=other,scope='two original synthetic images; no ROS/Gazebo/ports')
(artifact/'execution_v1/duplicate_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,indent=2))
