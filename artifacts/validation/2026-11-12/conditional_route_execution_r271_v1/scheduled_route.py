"""r266의 출력 검증은 그대로 두고 불필요한 보조 제출만 줄이는 시험 후보."""
from collections import Counter, deque
from pathlib import Path
import math
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'retained_route_execution_r266_v1'))
from route_delivery import RouteProcess
from arena_autonomy.decision_delivery import source_identity
from submission_policy import assess


def percentile(values, quantile):
    ordered = sorted(values)
    index = (len(ordered) - 1) * quantile
    left = int(index)
    return ordered[left] + (ordered[min(left + 1, len(ordered) - 1)] - ordered[left]) * (index - left)


class ScheduledRouteProcess(RouteProcess):
    def __init__(self, header, **kwargs):
        super().__init__(header, **kwargs)
        self.costs = {stage: deque(maxlen=32) for stage in ('plan', 'current')}
        self.last_command_reference = -math.inf
        self.schedule_counts = Counter()
        self.schedule_events = deque(maxlen=512)
        self._last_decision = None

    def collect(self):
        rows = super().collect()
        for row in rows:
            # 회수까지 실제 걸린 시간이며 미래 실행 결과를 사용하지 않는다.
            self.costs[row['stage']].append(row['received_wall_s'] - row['submitted_wall_s'])
        return rows

    def invalidate(self, reason=None):
        super().invalidate(reason)
        self.last_command_reference = -math.inf
        self._last_decision = None

    def commit_published(self, token, **state):
        reference = self.ready['output']['reference_s'] if self.ready else -math.inf
        super().commit_published(token, **state)
        self.last_command_reference = reference

    def offer_retained_if_useful(self, snapshot, *, now_s):
        # 새 계획·이미 진행/완료된 확인을 먼저 처리한다.
        if (self.closed or self.fault or self.saved_plan is not None or self.current.inflight is not None
                or self.ready is not None or not self._usable(self.retained)):
            return False
        if self.plan.inflight is None or any(len(v) < 3 for v in self.costs.values()):
            return False
        source = source_identity(snapshot)
        key = (self.retained['generation'], self.retained['sequence'], source)
        if key == self.last_checked_source:
            return False
        decision = assess(now_s=now_s, now_wall_s=time.monotonic(),
            plan_submitted_wall_s=self.plan.inflight['submitted_wall_s'],
            plan_budget_s=percentile(self.costs['plan'], .25),
            current_budget_s=percentile(self.costs['current'], .75),
            last_application_s=self.last_application,
            last_command_reference_s=self.last_command_reference,
            current_reference_s=snapshot['reference_s'], source_first_s=source[1],
            path_origin_s=self.retained['output'][2]['normal']['path_source_s'])
        identity = (key, self.plan.inflight['sequence'], decision['reason'])
        if identity != self._last_decision:
            self.schedule_counts[decision['reason']] += 1
            self.schedule_events.append(dict(now_s=now_s, source=source,
                plan_sequence=self.plan.inflight['sequence'], **decision))
            self._last_decision = identity
        if not decision['offer']:
            return False
        offered = super().offer_next_current(snapshot)
        self.schedule_counts['actually_offered'] += bool(offered)
        return offered
