"""센서 순서로 입구 검토 시점만 찾는다. 주행 출력·위치 지도·장애물 허가 없음."""
import math


class BranchIntent:
    def __init__(self):
        self.reset(0., 0)

    def reset(self, now, epoch):
        self.epoch, self.floor = epoch, now
        self.state, self.reason = 'idle', 'reset'
        self.last_frame, self.last_stamp, self.last_time = -math.inf, None, None
        self.arm_s, self.yaw, self.peak, self.was_left = None, 0., 0., False
        self.notch = None

    def _retire(self, reason):
        self.state, self.notch, self.reason = 'spent', None, reason

    def frame(self, ids, stamp, now, epoch):
        if (epoch != self.epoch or stamp is None or not math.isfinite(stamp) or
            not math.isfinite(now) or stamp < self.floor or stamp <= self.last_frame or not 0 <= now-stamp < .45):
            return
        self.last_frame = stamp
        if any(i in (0,30,45) for i in ids):
            self.state,self.notch,self.reason='idle',None,'other_course_marker'
            return
        if self.state=='idle' and 20 in ids:
            self.state,self.reason='armed','id20'
            self.arm_s,self.yaw,self.peak,self.was_left=now,0.,0.,False
            self.last_time,self.last_stamp,self.notch=None,None,None

    @staticmethod
    def as_pair(n):
        try:
            p=tuple(tuple(float(v) for v in n[k]) for k in ('e1','e2'))
            if any(len(point)!=2 or not all(map(math.isfinite,point)) for point in p):return None
            if not (.05<p[0][0]<3. and p[1][0]>p[0][0] and p[0][1]<0 and p[1][1]<0):return None
            if not .40 <= math.dist(*p) <= .86:return None
            return p
        except (KeyError, TypeError, ValueError, OverflowError):
            return None

    def scan(self, now, source, notches, motion, epoch):
        """motion: 직전 처리한 스캔 판단 시각에서 현재까지 차체 변위(옛 차체축)."""
        if epoch != self.epoch:
            self.reset(now,epoch)
            return False
        if self.state!='armed':return False
        if source < self.floor:return False
        if not all(map(math.isfinite,(now,source))) or not self.arm_s<=now<self.arm_s+8. or not 0<=now-source<.45:
            self._retire('timing');return False
        if source==self.last_stamp:return False
        if self.last_stamp is not None and (source<self.last_stamp or not 0<now-self.last_time<=.25):
            self._retire('scan_discontinuity');return False
        old=self.notch
        if self.last_time is not None:
            if motion is None or len(motion)!=3 or not all(map(math.isfinite,motion)):
                self._retire('motion_gap');return False
            dx,dy,da=motion
            self.yaw+=da
            self.peak=max(self.peak,self.yaw)
            if self.yaw < -math.radians(35) or self.yaw>math.radians(135) or self.peak-self.yaw>=math.radians(35):
                self._retire('wrong_or_next_turn');return False
            self.was_left |= self.yaw>=math.radians(60)
        else:
            dx=dy=da=0.
        paired_interval=self.last_time is not None and now-self.last_time<=.16
        self.last_stamp,self.last_time=source,now
        p=self.as_pair(notches[0]) if self.was_left and len(notches)==1 else None
        self.notch=p
        if p is None or old is None or not paired_interval:return False
        c,s=math.cos(da),math.sin(da)
        def moved(pt):
            x,y=pt[0]-dx,pt[1]-dy
            return c*x+s*y,-s*x+c*y
        if max(math.dist(moved(a),b) for a,b in zip(old,p))>.10:return False
        self._retire('observed_entry_review')
        return True
