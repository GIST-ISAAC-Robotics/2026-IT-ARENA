"""0~4개 점등 정지 이미지의 픽셀 변화. 실시간 순차 검출/출발 성공을 뜻하지 않는다."""
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

sources={}
rows=[]
for mount in ('rear_20cm','rear_30cm'):
    for count in range(5):
        directory=OUT/('render_h25_red0_env2' if count==0 else f'render_h25_red{count}')/'capture'
        report=json.loads((directory/'report.json').read_text(encoding='utf-8'))
        assert report['passed'] and report['source_unchanged']
        path=directory/(mount+'.png')
        rgb=cv2.cvtColor(cv2.imread(str(path)),cv2.COLOR_BGR2RGB)
        red=red_signal_mask(cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV));red[int(rgb.shape[0]*.58):]=0
        contours,_=cv2.findContours(red,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        components=[]
        for contour in contours:
            x,y,w,h=cv2.boundingRect(contour)
            region=red[y:y+h,x:x+w]
            components.append(dict(rect=[x,y,w,h],area_px=cv2.contourArea(contour),
                lit_pixels=int(np.count_nonzero(region)),
                row_pixel_counts=np.count_nonzero(region,axis=1).tolist(),
                column_pixel_counts=np.count_nonzero(region,axis=0).tolist()))
        rows.append(dict(mount=mount,red_count=count,components=components))
        for p in (path,directory/'report.json'):
            sources[p.relative_to(ROOT).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
for p in (Path(__file__),ROOT/'src/arena_autonomy/arena_autonomy/core.py'):
    sources[p.relative_to(ROOT).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
with (OUT/'red_growth_analysis.json').open('x',encoding='utf-8') as stream:
    json.dump(dict(scope='separate_static_material_stages_not_live_transition',source_sha256=sources,rows=rows),stream,indent=2)
print(json.dumps(rows))
