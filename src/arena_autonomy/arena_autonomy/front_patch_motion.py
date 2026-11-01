"""최근 세 관측의 보수적 전진율. 정지 누적·물체 신원·미관측 공간 판정과 분리한다."""
from collections import deque
import numpy as np
from arena_autonomy.scene_motion import transport_points


class RecentForward:
    def __init__(self,bound=.06,motion_bound=.5):
        self.bound=bound;self.motion_bound=motion_bound
        self.reset()

    def reset(self):
        self.rows=deque(maxlen=3);self.rate=0.

    def observe(self,source,t,xy,history):
        self.rate=0.
        if xy is None:
            self.reset();return
        if self.rows and not (0<source-self.rows[-1][0]<=.25 and t>self.rows[-1][1]):
            self.reset()
        self.rows.append((source,t,np.asarray(xy)))
        if len(self.rows)<3:return
        try:
            q=np.asarray([transport_points(p[None],history,ref,t)[0] if ref!=t else p for _,ref,p in self.rows])
        except ValueError:
            self.reset();return
        dt=np.diff([x[0] for x in self.rows]);ds=np.diff(q,axis=0)
        # 단일 튐/교체는 전진 속도가 아니다. 두 이동이 연속 전방이고 기존 연관 범위 안이어야 한다.
        if (not np.isfinite(q).all() or np.any(ds[:,0]<=np.abs(ds[:,1]))
                or np.any(np.linalg.norm(ds,axis=1)>self.bound+self.motion_bound*dt)):
            return
        # 두 끝점의 ±3cm를 먼저 뺀다. 변화량 전체를 상대의 정확한 속도로 해석하지 않는다.
        span=self.rows[-1][0]-self.rows[0][0]
        self.rate=min(self.motion_bound,max(0.,(q[-1,0]-q[0,0]-self.bound)/span))

