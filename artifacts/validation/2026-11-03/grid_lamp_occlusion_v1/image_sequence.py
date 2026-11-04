"""저장 Gazebo 픽셀에 이상적인 상대 외형을 투영한 입력. 실제 상대 주행/카메라 시퀀스가 아니다."""
import json
import sys
from pathlib import Path
import hashlib
import math
import xml.etree.ElementTree as ET
import cv2
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.race_start_local import LocalOfficialStartSignal
from arena_autonomy.core import marker_ids
from audit import GRID, numbers, lamps

RENDER=HERE.parents[1]/'2026-11-02/official_facility_v1/render_v1'


def project_prism(slot,height,translation,camera,shape):
    x,y,z,roll,pitch,yaw=camera
    assert abs(roll)+abs(pitch)<1e-9
    h,w=shape[:2]; a=slot['yaw_rad'];c,s=math.cos(a),math.sin(a)
    cw,sw=math.cos(yaw),math.sin(yaw);f=w/(2*math.tan(math.radians(75)/2))
    corners=[]
    for u in (-.1,.1):
        for v in (-.075,.075):
            for zz in (0.,height):
                dx=slot['x']+(u+translation)*c-v*s-x
                dy=slot['y']+(u+translation)*s+v*c-y
                depth=cw*dx+sw*dy
                # 5대 중 마지막 그리드에서 관측하므로 타 차량 전체가 카메라 앞이다.
                assert depth>0
                corners.append((w/2-f*(-sw*dx+cw*dy)/depth,h/2-f*(zz-z)/depth))
    return cv2.convexHull(np.rint(corners).astype(np.int32))


def main():
    fixture=RENDER/'fixture.sdf';root=ET.parse(fixture)
    model=root.find("world/model[@name='m5_camera_slot0']")
    camera=numbers(model,'pose',6); sensor=model.find('link/sensor')
    assert not any(numbers(model.find('link'),'pose',6)+numbers(sensor,'pose',6))
    assert abs(float(sensor.findtext('camera/horizontal_fov'))-math.radians(75))<1e-8
    g=sorted(json.loads(GRID.read_bytes())['starting_grid']['slots'],key=lambda q:-q['s_m'])[:5]
    assert g[-1]['index']==0
    files={k:RENDER/('slot0_'+k+'.png') for k in ('off','red4','green')}
    frames={k:cv2.imread(str(f)) for k,f in files.items()}
    assert all(a is not None and a.shape==(720,1280,3) for a in frames.values())
    def compose(stage,move):
        img=frames[stage].copy()
        for q in g[:-1]:  # 앞순서부터 덧그린다. 모든 상자는 같은 무광 색이라 겹침 순서와 무관하다.
            direct=q['index']==1
            poly=project_prism(q,.225 if direct else .12,move if direct else 0.,camera,img.shape)
            cv2.fillConvexPoly(img,poly,(96,96,96))
        return img
    stages={'original_red':frames['red4'],'original_green':frames['green'],'covered_red':compose('red4',0.),
            'covered_green':compose('green',0.),'moved_green':compose('green',.16),
            'moved_off':compose('off',.16)}
    # 같은 RGB 재질의 두 저장 단계에서 실제 달라지는 광원 픽셀만 가림 전후에 대조한다.
    # 컨트롤러에는 정답 마스크/차 위치/공식 상태를 주지 않는다.
    hsv=lambda im:cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    red=(cv2.inRange(hsv(frames['red4']),(0,130,176),(12,255,255))>0)
    green=(cv2.inRange(hsv(frames['green']),(41,131,176),(89,255,255))>0)
    red &= (np.max(cv2.absdiff(frames['red4'],frames['off']),axis=2)>30)
    green &= (np.max(cv2.absdiff(frames['green'],frames['off']),axis=2)>30)
    assert red.sum()>20 and green.sum()>20
    def count(im,mask,original):
        return int(np.count_nonzero(mask&np.all(im==original,axis=2)))
    visible={'red_total':int(red.sum()),'covered_red_unchanged':count(stages['covered_red'],red,frames['red4']),
             'green_total':int(green.sum()),'covered_green_unchanged':count(stages['covered_green'],green,frames['green']),
             'moved_green_unchanged':count(stages['moved_green'],green,frames['green'])}
    seqs={'normal':['original_red']*2+['original_green']*3,
          'hidden_then_green':['covered_red']*2+['covered_green']*2+['moved_green']*3,
          'front_moved_signal_off':['covered_red']*2+['moved_off']*3}
    calls={}
    for name,stream in seqs.items():
        gate=LocalOfficialStartSignal();items=[]
        for k,key in enumerate(stream):
            rgb=cv2.cvtColor(stages[key],cv2.COLOR_BGR2RGB)
            started=gate.update(rgb,1.+k/30)
            items.append(dict(frame=key,started=bool(started),state=gate.observed,
                              markers=sorted(set(marker_ids(rgb,.02)))))
        calls[name]=items
    for key in ('covered_red','moved_green'):
        path=HERE/(key+'.png')
        assert not path.exists() and cv2.imwrite(str(path),stages[key])
    result=dict(source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in [fixture,GRID,*files.values()]},
                ego=0,front=1,relative_front_motion_m=.16,heights_m=dict(leading=.225,others=.12),
                visibility=visible,sequences=calls,
                scope='11/2 기둥 포함 고정 렌더 프레임과 명목 상자 투영. 서로 다른 시각을 부여한 반복 픽셀은 카메라 연속 프레임/실제 차량 동작이 아니다. 정답 기하는 이미지 생성·평가에만 사용.')
    with (HERE/'sequence.json').open('x',encoding='utf8') as f:json.dump(result,f,indent=2,ensure_ascii=False)
    print(visible,{k:v[-1]['state'] for k,v in calls.items()})
    assert (visible['covered_red_unchanged']==visible['covered_green_unchanged']==0
            and visible['moved_green_unchanged']==visible['green_total'] and calls['normal'][-1]['started']
            and not any(q['started'] for k in ['hidden_then_green','front_moved_signal_off'] for q in calls[k]))


if __name__=='__main__':
    main()
