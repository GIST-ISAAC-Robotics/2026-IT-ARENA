"""계획/현재 검사 후보의 발행 인계. 실제 ROS 발행은 호출자 책임이다.

retained는 계산이 끝난 경로가 아니라 commit_published로 확인된 경로만 쓴다.
선택형 시험 코드이며 공통 런타임/기본 정책에는 연결하지 않았다.
"""
import copy
import math

import candidate_policy
from arena_autonomy.decision_delivery import source_identity
from overlap import OverlapProcess, plan_reference


class RouteProcess(OverlapProcess):
    def __init__(self, header, **kwargs):
        super().__init__(header, **kwargs)
        self.retained = None
        self.ready = None
        self.revision = 0
        self.last_checked_source = None
        self.last_application = -math.inf
        self.geometry_age_limit = float(header['follow_config'].get('memory_ttl_s', .3)) + .1

    def invalidate(self, reason=None):
        super().invalidate(reason)
        self.retained = self.ready = self.last_checked_source = None
        self.revision += 1

    def _usable(self, plan):
        return (plan is not None and plan['generation'] == self.generation
                and candidate_policy.supported(plan['output']) and plan['output'][0] > 0)

    def collect(self):
        rows = super().collect()
        # 한 회수에서 새 계획과 현재 검사가 같이 도착하면 계획 전이를 먼저 반영한다.
        for row in rows:
            if row['stage'] == 'plan':
                self.revision += 1
                self.ready = None
                if not self._usable(row):
                    self.retained = None
        for row in rows:
            if row['stage'] != 'current':
                continue
            metadata = row['plan'].get('route_check') or {}
            row['route_check'] = copy.deepcopy(metadata)
            if metadata.get('revision') != self.revision:
                row['discard_reason'] = 'new_plan_superseded_current_check'
                continue
            row['generation'] = self.generation
            # 보조 검사 불성립은 새 명령을 만들지 않는다. 구 명령의 시각도 연장하지 않는다.
            if metadata.get('retained') and row['output']['speed_mps'] <= 0:
                row['discard_reason'] = 'retained_route_not_proven'
                continue
            if row['output']['speed_mps'] <= 0:
                self.retained = None
            self.ready = copy.deepcopy(row)
        if self.fault:
            self.retained = self.ready = None
        return rows

    def offer_next_current(self, snapshot):
        if self.closed or self.fault:
            raise RuntimeError('route process unavailable')
        if self.current.inflight is not None or self.ready is not None:
            return False
        source = source_identity(snapshot)
        if source == self.last_checked_source:
            return False
        # 완료된 새 계획을 먼저 처리한다. 새 0/상태 전이는 보조 경로보다 우선한다.
        retained = self.saved_plan is None
        plan = self.retained if retained else self.saved_plan
        if not self._usable(plan):
            return False
        if snapshot['reference_s'] < plan['input']['reference_s']:
            raise ValueError('current snapshot precedes original route')
        owned_plan = copy.deepcopy(plan)
        owned = copy.deepcopy(snapshot)
        owned_plan['route_check'] = dict(retained=retained, revision=self.revision)
        data = dict(output=owned_plan['output'], submitted=plan_reference(owned_plan['input']),
                    current=owned, retained=retained)
        self.sequence += 1
        self.current.offer(data, self.generation, self.sequence)
        self.current_input = dict(plan=owned_plan, current=owned)
        self.last_checked_source = source
        if not retained:
            self.saved_plan = None
        return True

    def offer_current(self, *args, **kwargs):
        raise RuntimeError('use route-owned offer_next_current')

    def application_reason(self, row, *, now_s, previous_steering_rad, side,
                           motion_epoch, sensor_epoch):
        if (self.ready is None or row.get('sequence') != self.ready['sequence']
                or row.get('generation') != self.generation
                or row.get('route_check', {}).get('revision') != self.revision):
            return 'candidate_superseded'
        # 외부 사본의 출력 변경은 발행 권한이 아니다. 내부 소유 후보로 판정한다.
        row = self.ready
        old = row['plan']['input']; cur = row['input']; out = row['output']
        if not math.isfinite(now_s) or now_s < cur['reference_s']:
            return 'current_clock_boundary'
        if out.get('reference_s') != cur['reference_s']:
            return 'current_reference_changed'
        for data in (old, cur):
            epoch, _ = source_identity(data)
            if data['epoch'] != motion_epoch or epoch != sensor_epoch or data['side'] != side:
                return 'current_input_epoch_or_side_changed'
        if now_s - source_identity(cur)[1] >= .3:
            return 'current_source_expired'
        if now_s - cur['reference_s'] >= .2:
            return 'current_command_expired'
        if previous_steering_rad != cur['previous_steering_rad']:
            return 'current_previous_request_changed'
        if out['speed_mps'] <= 0:
            return None
        origin = row['plan']['output'][2]['normal']['path_source_s']
        if not math.isfinite(origin) or not 0 <= origin <= old['reference_s'] <= cur['reference_s']:
            return 'route_source_invalid'
        if now_s - origin >= self.geometry_age_limit:
            return 'route_geometry_expired'
        proof = out.get('current_route_proof') or {}
        needs_proof = (row['route_check']['retained']
                       or now_s - source_identity(old)[1] >= .3
                       or now_s - old['reference_s'] >= .2)
        if needs_proof and not (
                proof.get('accepted') is True
                and proof.get('original_path_source_s') == origin
                and proof.get('current_reference_s') == cur['reference_s']
                and proof.get('current_evidence_first_s') == source_identity(cur)[1]
                and proof.get('source_timestamp_renewed') is False
                and proof.get('old_occupancy_permission_reused') is False):
            return 'new_current_route_proof_required'
        if now_s - self.last_application < .1 - 1e-9:
            return 'positive_application_interval'
        return None

    def prepared(self, **state):
        if self.ready is None:
            return None
        reason = self.application_reason(self.ready, **state)
        if reason:
            # 간격만 부족하면 같은 후보를 잠시 보관하고 적용 직전에 다시 검사한다.
            if reason != 'positive_application_interval':
                self.ready = None
            return dict(kind='not_applied', reason=reason)
        return dict(kind='candidate', result=copy.deepcopy(self.ready))

    def commit_published(self, token, **state):
        """동일 콜백의 /drive 발행 직후 호출한다. 실제 발행 여부를 대신 입증하지 않는다."""
        row = self.ready
        if row is None or token != (row['generation'], row['sequence']):
            raise ValueError('single current candidate token required')
        reason = self.application_reason(row, **state)
        if reason:
            raise ValueError('candidate not applicable: ' + reason)
        self.last_application = state['now_s']
        self.retained = copy.deepcopy(row['plan']) if row['output']['speed_mps'] > 0 else None
        if self.retained is not None:
            self.retained.pop('route_check', None)
        self.ready = None

    def discard_ready(self):
        self.ready = None
