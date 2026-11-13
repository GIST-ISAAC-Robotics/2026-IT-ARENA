"""현재 면의 전진이 확인될 때만 양보 예산을 갱신하는 선택형 v3 후보.

5초를 무조건 늘리지 않는다. 기존 면·벽·시각 검사를 모두 통과한 새 스캔에서
전방 중앙점이 마지막 갱신점보다 10 cm 앞서면 감속 예산만 갱신한다.
10 cm는 두 끝점의 6 cm 거리 불확실성보다 큰 시험 여유이며 실측 오차 보장은 아니다.
복귀 허가·점유·최종 명령·짧은 운동 대기의 1초 제한은 바꾸지 않는다.
"""
from arena_autonomy.bypass_relative_yield import BoundedRelativeYield
from arena_autonomy.bypass_relative_yield_pause import MotionPauseRelativeYieldCommand


class ProgressRelativeYield(BoundedRelativeYield):
    minimum_forward_progress_m = .10

    def reset(self):
        super().reset()
        self.progress_mark = None
        self.started_source = None
        self.renewals = 0

    def update(self, row):
        result = super().update(row)
        if result.get('suggest_yield') and self.progress_mark is None:
            self.progress_mark = (row['along_low'] + row['along_high']) / 2.
            self.started_source = self.yield_since
        return result

    def continue_observation(self, row, *, motion_pause_gap=False):
        previous_source = self.key[1] if self.key else None
        result = super().continue_observation(row, motion_pause_gap=motion_pause_gap)
        if not result.get('suggest_yield'):
            return result
        center = (row['along_low'] + row['along_high']) / 2.
        progressed = (self.progress_mark is not None and row['source'] > previous_source
            and center - self.progress_mark >= self.minimum_forward_progress_m)
        # 현재 면의 연속 관측만 사용한다. 무반환·다른 면·시각 역전은 부모 검사에서 해제된다.
        if progressed:
            self.progress_mark = center
            self.yield_since = row['source']
            self.renewals += 1
        self.detail = dict(result, progress_budget_renewed=bool(progressed),
            progress_budget_renewals=self.renewals, progress_mark_m=self.progress_mark,
            budget_age_s=row['now'] - self.yield_since,
            total_yield_s=row['now'] - self.started_source,
            budget_scope='fresh_observed_forward_progress_only')
        return self.detail


class ProgressRelativeYieldCommand(MotionPauseRelativeYieldCommand):
    def __init__(self):
        super().__init__()
        self.observer = ProgressRelativeYield()
