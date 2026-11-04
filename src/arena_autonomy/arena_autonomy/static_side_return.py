"""측방 반환이 세계에 고정돼 보이는지 판단하는 선택형 근거.

자차가 이동한 거리만큼 경로 좌표에서 뒤로 물러나는 반환만 인정한다. 정지·물체 신원의
증명이 아니며 경로 점유 검사와 독립 보호층을 대신하지 않는다. 함께 달리거나 앞지르는
차는 이동량과 어긋나므로 기존 측방 위협으로 남는다.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class StaticSideConfig:
    minimum_scans: int = 3
    minimum_travel_m: float = .12    # 스캔 하나의 각도 양자화보다 큰 자차 이동
    minimum_span_s: float = .2
    tolerance_m: float = .05         # C1 거리 오차 약 3 cm와 끝점 양자화 여유
    maximum_gap_s: float = .25
    window: int = 8


class StaticSideTracker:
    def __init__(self, config=None):
        self.config = config or StaticSideConfig()
        self.reset()

    def reset(self):
        self.travel_m = 0.
        self.rows = []

    def advance(self, speed, dt):
        self.travel_m += max(float(speed), 0.)*float(dt)

    def observe(self, stamp, front_ds):
        """같은 위치에 남아 있는 앞끝을 누적한다. 어긋나면 새 근거부터 다시 모은다."""
        cfg = self.config
        if front_ds is None:
            self.rows = []
            return False
        if self.rows and stamp-self.rows[-1][0] > cfg.maximum_gap_s:
            self.rows = []
        row = (stamp, float(front_ds), self.travel_m)
        if self.rows:
            first = self.rows[0]
            if abs(row[1]-first[1]+row[2]-first[2]) > cfg.tolerance_m:
                self.rows = []
        self.rows.append(row)
        self.rows = self.rows[-cfg.window:]
        first = self.rows[0]
        return (len(self.rows) >= cfg.minimum_scans and row[2]-first[2] >= cfg.minimum_travel_m
                and row[0]-first[0] >= cfg.minimum_span_s)
