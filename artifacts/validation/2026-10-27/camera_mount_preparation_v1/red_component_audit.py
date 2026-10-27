"""이미 촬영한 빨강 연결 성분의 크기를 확인한다. 재렌더링·검출기 수정 없음."""
from pathlib import Path
import hashlib
import json
import sys
import cv2

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
sys.path.insert(0,str(ROOT/'src/arena_autonomy'))
from arena_autonomy.core import red_signal_mask

rows=[]
sources={}
for h in (25,40):
    for name in ('rear_20cm','rear_30cm'):
        path=OUT/f'render_h{h}_red4/capture/{name}.png'
        rgb=cv2.cvtColor(cv2.imread(str(path)),cv2.COLOR_BGR2RGB)
        mask=red_signal_mask(cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV))
        mask[int(rgb.shape[0]*.58):]=0
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        components=[]
        for contour in contours:
            x,y,w,height=cv2.boundingRect(contour)
            components.append(dict(rect=[x,y,w,height],area_px=cv2.contourArea(contour),
                aspect=w/height,existing_width_pass=5<=w<=100,existing_height_pass=4<=height<=100,
                existing_aspect_pass=.55<w/height<1.8,existing_area_pass=cv2.contourArea(contour)>10))
        rows.append(dict(actor_height_m=h/100,mount=name,components=components))
        sources[path.relative_to(ROOT).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
sources[Path(__file__).relative_to(ROOT).as_posix()]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
with (OUT/'red_component_audit.json').open('x') as stream:
    json.dump(dict(scope='existing_red_mask_components_not_new_detector',source_sha256=sources,rows=rows),stream,indent=2)
print(json.dumps(rows))
