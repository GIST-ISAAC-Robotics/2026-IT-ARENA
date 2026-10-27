"""정지 영상의 점등 증가 방향과 초록 위치 대조. 출발 허가/실시간 검출기 아님."""
import hashlib
import json
from pathlib import Path
import sys
import cv2
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
sys.path.insert(0,str(ROOT/'src/arena_autonomy'))
from arena_autonomy.core import red_signal_mask

target=OUT/'red_increment_geometry.json'
if target.exists():raise FileExistsError(target)
hashes={};rows=[]
def load(directory,mount):
    path=OUT/directory/'capture'/f'{mount}.png'
    hashes[path.relative_to(ROOT).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    bgr=cv2.imread(str(path));assert bgr is not None
    return cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
def centroid(mask):
    y,x=np.nonzero(mask)
    return np.array([x.mean(),y.mean()]) if len(x) else None

for mount in ('rear_20cm','rear_30cm'):
    masks=[]
    for count in range(1,5):
        hsv=load(f'render_h25_red{count}',mount)
        mask=red_signal_mask(hsv)>0;mask[int(mask.shape[0]*.58):]=False
        masks.append(mask)
    increments=[masks[0]]+[current & ~previous for previous,current in zip(masks,masks[1:])]
    centers=np.array([centroid(m) for m in increments])
    coefficients=np.linalg.lstsq(np.column_stack((np.ones(4),np.arange(4))),centers,rcond=None)[0]
    fitted=coefficients[0]+np.arange(4)[:,None]*coefficients[1]
    predicted_green=coefficients[0]+4*coefficients[1]
    green=load('render_h25_green',mount)
    gm=cv2.inRange(green,(41,131,176),(89,255,255))>0;gm[int(gm.shape[0]*.58):]=False
    green_center=centroid(gm)
    # 노출 증가에 따른 대칭 팽창 대조. 영상 재질/실제 노출 현상 재현은 아니다.
    dilation=[cv2.dilate(masks[0].astype(np.uint8),np.ones((k,k),np.uint8))>0 for k in (1,3,5,7)]
    symmetric=[dilation[0]]+[b & ~a for a,b in zip(dilation,dilation[1:])]
    symmetric_centers=np.array([centroid(m) for m in symmetric])
    rows.append(dict(mount=mount,increment_pixels=[int(m.sum()) for m in increments],
        retained_previous_fractions=[float(np.count_nonzero(a&b)/np.count_nonzero(a)) for a,b in zip(masks,masks[1:])],
        increment_centers_xy=centers.tolist(),fitted_step_xy_px=coefficients[1].tolist(),
        maximum_center_residual_px=float(np.max(np.linalg.norm(centers-fitted,axis=1))),
        predicted_green_xy=predicted_green.tolist(),observed_green_xy=green_center.tolist(),
        green_error_px=float(np.linalg.norm(predicted_green-green_center)),
        symmetric_dilation_increment_centers_xy=symmetric_centers.tolist(),
        symmetric_dilation_center_span_px=float(np.linalg.norm(np.ptp(symmetric_centers,axis=0)))))
for p in (Path(__file__),ROOT/'src/arena_autonomy/arena_autonomy/core.py'):
    hashes[p.relative_to(ROOT).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
result=dict(scope='offline_static_image_increment_geometry_not_start_permission',source_sha256=hashes,rows=rows,
    limits=['separate static captures with fixed camera are not sequential live frames',
            'no head movement, exposure control, packet timing, or actual opponents validated',
            'same color pattern can still be mimicked by other objects; not proof of signal identity',
            'red-occluded then green-only start remains unresolved'])
with target.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
print(json.dumps(rows))
