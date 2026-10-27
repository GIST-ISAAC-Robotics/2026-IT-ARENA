"""순차 증가 후보의 정지 영상 대조. 운영 노드에서 import하지 않는 시제품."""
import hashlib
import json
import math
from pathlib import Path
import sys
import cv2
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
sys.path.insert(0,str(ROOT/'src/arena_autonomy'))
from arena_autonomy.core import red_signal_mask


class GrowthProbe:
    """네 단계의 동일 방향 증가 후 예측 위치의 초록만 확인한다. 실물 사양 아님."""
    def __init__(self):
        self.last_stamp=None;self.started=False;self.forget()

    def forget(self):
        self.level=0;self.mask=None;self.centers=[];self.unit=None;self.diameter=None
        self.pending=None;self.confirmations=0;self.green_frames=0;self.last_red=None
        self.reason='waiting_growth'

    @staticmethod
    def center(mask):
        y,x=np.nonzero(mask)
        return np.array([x.mean(),y.mean()]) if len(x) else None

    def update(self,rgb,stamp):
        if not math.isfinite(stamp) or stamp<0:
            self.__init__();self.reason='invalid_stamp';return False
        if self.last_stamp is not None and stamp<=self.last_stamp:
            if stamp<self.last_stamp:self.__init__();self.reason='clock_reset'
            return self.started
        if self.last_stamp is not None and stamp-self.last_stamp>.35:self.forget()
        self.last_stamp=stamp
        if self.started:return True
        hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
        red=red_signal_mask(hsv)>0;red[int(rgb.shape[0]*.58):]=False
        area=int(red.sum())
        if area:
            self.green_frames=0
            if self.mask is not None and np.array_equal(self.mask,red):
                self.last_red=stamp;self.reason=f'red{self.level}_stable';return False
            if self.pending is None or not np.array_equal(self.pending,red):
                self.pending=red.copy();self.confirmations=1;self.reason='confirming_growth';return False
            self.confirmations+=1
            if self.confirmations<2:return False
            if self.level==0:
                n,_,stats,_=cv2.connectedComponentsWithStats(red.astype(np.uint8),8)
                if n!=2:
                    self.forget();self.reason='seed_ambiguous';return False
                _,_,w,h,_=stats[1]
                if min(w,h)<4 or max(w,h)>100 or not .55<w/h<1.8:
                    self.forget();self.reason='seed_shape';return False
                self.unit=area;self.diameter=(w+h)/2;center=self.center(red)
            else:
                retained=np.count_nonzero(red&self.mask)/np.count_nonzero(self.mask)
                added=red & ~self.mask;increase=int(added.sum());center=self.center(added)
                if self.level>=4 or retained<.85 or not .5*self.unit<=increase<=1.4*self.unit:
                    self.forget();self.reason='inconsistent_increment';return False
                step=center-self.centers[-1]
                if not .45*self.diameter<=np.linalg.norm(step)<=1.5*self.diameter:
                    self.forget();self.reason='increment_not_directional';return False
                if len(self.centers)>1:
                    expected=(self.centers[-1]-self.centers[0])/(len(self.centers)-1)
                    if np.linalg.norm(step-expected)>max(1.5,.35*np.linalg.norm(expected)):
                        self.forget();self.reason='increment_changed_direction';return False
            self.centers.append(center);self.mask=red.copy();self.level+=1;self.last_red=stamp
            self.pending=None;self.confirmations=0;self.reason=f'red{self.level}_confirmed';return False
        self.pending=None;self.confirmations=0
        if self.level!=4:
            self.forget();self.reason='incomplete_sequence';return False
        if stamp-self.last_red>.8:
            self.forget();self.reason='transition_expired';return False
        centers=np.array(self.centers)
        fit=np.linalg.lstsq(np.column_stack((np.ones(4),np.arange(4))),centers,rcond=None)[0]
        expected=fit[0]+4*fit[1]
        green=cv2.inRange(hsv,(41,131,176),(89,255,255))
        green[int(rgb.shape[0]*.58):]=0
        n,_,stats,centroids=cv2.connectedComponentsWithStats(green,8)
        matched=[]
        for i in range(1,n):
            if (.4*self.unit<=stats[i,cv2.CC_STAT_AREA]<=1.8*self.unit and
                    np.linalg.norm(centroids[i]-expected)<=max(1.5,.5*self.diameter)):
                matched.append(i)
        self.green_frames=self.green_frames+1 if len(matched)==1 else 0
        self.started=self.green_frames>=3
        self.reason='green_confirmed' if self.started else 'confirming_green'
        return self.started


def main():
    target=OUT/'temporal_growth_probe.json'
    if target.exists():raise FileExistsError(target)
    hashes={};rows=[]
    for mount in ('rear_20cm','rear_30cm'):
        images=[]
        for stage in ('red1','red2','red3','red4','green'):
            path=OUT/f'render_h25_{stage}'/'capture'/f'{mount}.png'
            hashes[path.relative_to(ROOT).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
            images.append(cv2.cvtColor(cv2.imread(str(path)),cv2.COLOR_BGR2RGB))
        normal=[image for image in images[:4] for _ in range(3)]+[images[4]]*3
        wrong_green=np.roll(images[4],30,axis=1)
        simultaneous=images[3].copy();green_mask=cv2.inRange(cv2.cvtColor(images[4],cv2.COLOR_RGB2HSV),(41,131,176),(89,255,255))>0
        simultaneous[green_mask]=images[4][green_mask]
        sequences={'complete':normal,'green_only':[images[4]]*8,'red4_only':[images[3]]*3+[images[4]]*3,
            'missing_red2':[images[0]]*3+[images[2]]*3+[images[3]]*3+[images[4]]*3,
            'red1_flicker':[images[0]]*3+[images[4]]*3,
            'wrong_green_location':normal[:-3]+[wrong_green]*3,
            'red_still_on':normal[:-3]+[simultaneous]*3,
            'duplicate_green_stamp':normal,'frame_gap_before_green':normal,'clock_reset_before_green':normal}
        for name,frames in sequences.items():
            p=GrowthProbe();details=[]
            for i,image in enumerate(frames):
                stamp=i*.1
                if i>=12 and name=='duplicate_green_stamp':stamp=1.2
                if i>=12 and name=='frame_gap_before_green':stamp+=1.
                if i>=12 and name=='clock_reset_before_green':stamp-=1.2
                started=p.update(image,stamp)
                details.append(dict(frame=i,assigned_stamp_s=stamp,started=started,reason=p.reason,level=p.level))
            rows.append(dict(mount=mount,case=name,expected_start=name=='complete',started=p.started,details=details))
    hashes[Path(__file__).relative_to(ROOT).as_posix()]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    result=dict(scope='prototype_repeated_static_frames_with_assigned_time_not_live_start',
        runtime_imported=False,passed=all(r['started']==r['expected_start'] for r in rows),rows=rows,source_sha256=hashes)
    with target.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
    print(json.dumps({'passed':result['passed'],'rows':[{k:r[k] for k in ('mount','case','expected_start','started')} for r in rows]}))


if __name__=='__main__':main()
