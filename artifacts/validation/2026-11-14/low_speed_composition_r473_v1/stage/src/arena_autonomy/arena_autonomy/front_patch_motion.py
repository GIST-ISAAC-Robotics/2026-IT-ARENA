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


class ObservedForwardPace:
    """짧은 연속 전방 반환의 전진율. 미래 우회 여유의 추가 속도 제한에만 쓴다.

    정지 관측 창은 빠른 앞차에서 매 스캔 초기화되므로 별도 최대 5스캔을 쓴다.
    기존 거리 비교 오차 6 cm를 끝점 이동에서 빼고 현재 G1 속도 상한으로 자른다.
    이 값은 물체 신원/미래 속도 보장이 아니며 점유·제동·독립 보호를 풀지 않는다.
    """
    def __init__(self, bound=.06, speed_limit=1.5):
        if not np.isfinite(bound) or bound <= 0 or not np.isfinite(speed_limit) or speed_limit <= 0:
            raise ValueError('finite positive observed pace bounds required')
        self.bound = bound
        self.speed_limit = speed_limit
        self.reset()

    def reset(self):
        self.rows = deque(maxlen=5)
        self.rate = 0.

    def observe(self, source, t, xy, history):
        self.rate = 0.
        if xy is None or not np.isfinite(xy).all():
            self.reset()
            return
        if self.rows and not (0 < source-self.rows[-1][0] <= .25 and t > self.rows[-1][1]):
            self.reset()
        self.rows.append((source, t, np.asarray(xy).copy()))
        if len(self.rows) < 3:
            return
        try:
            q = np.asarray([transport_points(p[None], history, ref, t)[0] if ref != t else p
                            for _, ref, p in self.rows])
        except ValueError:
            self.reset()
            return
        dt = np.diff([row[0] for row in self.rows])
        ds = np.diff(q, axis=0)
        # 전진과 구별되지 않는 측방 진입, 큰 점프, 정지/역방향은 속도 근거로 보태지 않는다.
        if (not np.isfinite(q).all() or np.any(ds[:, 0] <= np.abs(ds[:, 1]))
                or np.any(np.linalg.norm(ds, axis=1) > self.bound+self.speed_limit*dt)):
            return
        span = self.rows[-1][0]-self.rows[0][0]
        self.rate = max(0., min(self.speed_limit, (q[-1, 0]-q[0, 0]-self.bound)/span,
                               ds[-1, 0]/dt[-1]))

