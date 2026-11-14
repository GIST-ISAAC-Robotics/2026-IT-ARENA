"""r334의 두 슬롯 경로 인계 계약. 현재 검사 정책은 외부에서 명시한다.

원천300ms·명령200ms·경로500ms를 유지한다. 기본 양수발행은100ms이며,
명시한 실제 출력 dt의 current 결과만20ms 이후 조기 전달할 수 있다.
새로운 시각·점유 허가·독립 보호 통과를 만들지 않는다.
"""
import copy
import math
from arena_autonomy.decision_delivery import source_identity
from arena_autonomy.runtime_dual_process import DualSlots, plan_reference


def _checked_angle_fits_actual_interval(output, previous, output_dt_s, elapsed_s):
    """기존50Hz control 하한과 발행 직전의 같은3rad/s clamp를 유지한다."""
    values = (previous, output_dt_s, elapsed_s, output.get('steering_rad'))
    if not all(type(v) in (float, int) and math.isfinite(v) for v in values):
        return False
    if elapsed_s < .02-1e-9 or not 0 < output_dt_s <= min(.1, elapsed_s):
        return False
    return previous-3*output_dt_s <= output['steering_rad'] <= previous+3*output_dt_s


def current_request(output, submitted, current, *, retained, crop_current_selection=False):
    """부모 소유 입력에서 만든다. 새 scan 비교는 동일성뿐이며 허가가 아니다."""
    data = dict(output=output, submitted=plan_reference(submitted), current=current, retained=retained)
    if crop_current_selection:
        # frozen 사전의 비유한값 태그/원 배열/metadata 전체를 직접 비교한다.
        # 원 scan을 IPC에 중복 첨부하거나 매 호출 hash/JSON을 다시 만들지 않는다.
        data['crop_current_input'] = dict(schema='parent_owned_frozen_scan_equality_v1',
            same_scan=submitted['scan'] == current['scan'],
            submitted_reference_s=submitted['reference_s'], current_reference_s=current['reference_s'],
            submitted_timing_verified=submitted.get('timing_verified'),
            submitted_timing_policy=submitted.get('timing_policy', 'verified_only'),
            submitted_time_basis=submitted['scan'].get('measurements', {}).get('time_basis'))
    return data


def may_finish_current(old,new):
    """같은 FOLLOW 의도·비감속 새 양수 계획만 현재 검사 종료까지 보관한다.

    현재 경로의 점유 허가는 재사용하지 않는다. 실제 적용에는 원 RouteProcess의
    현재 관측/시각/기하/직전 조향 검사가 여전히 필요하다.
    """
    if old is None or new is None or old['generation']!=new['generation']:
        return False
    a,b=old['input'],new['input']
    if (a['epoch']!=b['epoch'] or source_identity(a)[0]!=source_identity(b)[0]
            or a['side']!=b['side'] or b['reference_s']<a['reference_s']):
        return False
    x,y=old['output'],new['output']
    if not all(math.isfinite(v) for v in (x[0],y[0])) or not 0<x[0]<=y[0]:
        return False
    for d in (x[2],y[2]):
        if d.get('handoff_state')!='FOLLOW' or d.get('reason')!='handoff_follow':
            return False
    fields=('mode','preferred_wall','path_wall','selector_mode','selector_choice')
    nx,ny=x[2].get('normal',{}),y[2].get('normal',{})
    if any(not isinstance(nx.get(k),str) or nx[k]!=ny.get(k) for k in fields):
        return False
    return nx['selector_mode']=='SOLO' and nx['selector_choice'] in ('baseline','parallel')


def same_positive_intent(old, new):
    if old is None or new is None:
        return False
    x, y = old['output'][0], new['output'][0]
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or v <= 0
           for v in (x, y)):
        return False
    # 속도 비교만 분리한다. 원 입력/경로를 바꾸거나 이 사본을 실행하지 않는다.
    probe = {**old, 'output': [min(x, y), *old['output'][1:]]}
    return may_finish_current(probe, new)


class RuntimeDualProcess(DualSlots):
    current_schedule_budget_s = .100
    retained_schedule_budget_s = .100

    def __init__(self, header, *, supported, **kwargs):
        if not callable(supported):
            raise ValueError('explicit current-check support predicate required')
        self.supported = supported
        self.crop_current_selection = (header.get('follow_config', {}).get('crop_edge_component_trial') is True
                                       and header.get('current_corner_budget') == 'r396_v1')
        super().__init__(header, **kwargs)
        self.retained = self.ready = self.last_checked_source = None
        self.revision = 0
        self.last_application = -math.inf
        self.geometry_age_limit = .5
        self.pending_speed_cap = None

    def invalidate(self, reason=None):
        super().invalidate(reason)
        self.retained = self.ready = self.last_checked_source = None
        self.revision += 1
        self.pending_speed_cap = None

    def _usable(self, plan):
        return (plan is not None and plan['generation'] == self.generation
                and self.supported(plan['output']) and plan['output'][0] > 0)

    def _cap_ready(self):
        cap = self.pending_speed_cap
        if cap is None or self.ready is None or self.ready['output']['speed_mps'] <= 0:
            return
        out = self.ready['output']
        checked_speed = (out.get('new_plan_deceleration_cap') or {}).get('checked_target_mps', out['speed_mps'])
        out['speed_mps'] = min(out['speed_mps'], cap['speed_mps'])
        out['new_plan_deceleration_cap'] = dict(
            **cap, checked_target_mps=checked_speed,
            applied=out['speed_mps'] < checked_speed,
            steering_changed=False, current_check_reused_without_timestamp_renewal=True)

    def collect(self):
        active = (self.current_input or {}).get('plan')
        if active is None and self.ready is not None:
            active = self.ready['plan']
        rows = DualSlots.collect(self)
        for row in rows:
            if row['stage'] != 'plan':
                continue
            if self._usable(row) and same_positive_intent(active, row):
                row['deferred_positive_transition'] = True
                speed = row['output'][0]
                old_cap = self.pending_speed_cap
                if speed < active['output'][0] and (old_cap is None or speed < old_cap['speed_mps']):
                    self.pending_speed_cap = dict(speed_mps=speed, generation=row['generation'],
                        sequence=row['sequence'], reference_s=row['input']['reference_s'])
            else:
                self.revision += 1
                self.ready = None
                self.pending_speed_cap = None
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
            if metadata.get('retained') and row['output']['speed_mps'] <= 0:
                row['discard_reason'] = 'retained_route_not_proven'
                continue
            if row['output']['speed_mps'] <= 0:
                self.retained = None
            self.ready = copy.deepcopy(row)
        self._cap_ready()
        if self.fault:
            self.retained = self.ready = self.pending_speed_cap = None
        return rows

    def offer_next_current(self, snapshot, *, now_s):
        ref = snapshot['reference_s']
        first = source_identity(snapshot)[1]
        if not all(math.isfinite(v) for v in (now_s, ref, first)) or now_s < ref:
            return False
        if now_s-ref+self.current_schedule_budget_s >= .2:
            return False
        if now_s-first+self.current_schedule_budget_s >= .3:
            return False
        if ref <= self.last_application:
            return False
        if self.saved_plan is None and self.retained is not None:
            source = self.retained['output'][2]['normal'].get('path_source_s')
            if not isinstance(source, (int, float)) or not math.isfinite(source):
                return False
            if ref-source+self.retained_schedule_budget_s >= self.geometry_age_limit:
                return False
        if self.closed or self.fault:
            raise RuntimeError('route process unavailable')
        if self.current.inflight is not None or self.ready is not None:
            return False
        source = source_identity(snapshot)
        # 완료된 새 계획을 먼저 처리한다. 새 0/상태 전이는 보조 경로보다 우선한다.
        retained = self.saved_plan is None
        plan = self.retained if retained else self.saved_plan
        if not self._usable(plan):
            return False
        source_key = (plan['generation'], plan['sequence'], source)
        if source_key == self.last_checked_source:
            return False
        if snapshot['reference_s'] < plan['input']['reference_s']:
            raise ValueError('current snapshot precedes original route')
        owned_plan = copy.deepcopy(plan)
        owned = copy.deepcopy(snapshot)
        owned_plan['route_check'] = dict(retained=retained, revision=self.revision)
        data = current_request(owned_plan['output'], owned_plan['input'], owned, retained=retained,
                               crop_current_selection=self.crop_current_selection)
        self.sequence += 1
        self.current.offer(data, self.generation, self.sequence)
        self.current_input = dict(plan=owned_plan, current=owned)
        self.last_checked_source = source_key
        if not retained:
            self.saved_plan = None
        self.pending_speed_cap = None
        return True

    def offer_current(self, *args, **kwargs):
        raise RuntimeError('use route-owned offer_next_current')

    def application_reason(self, row, *, now_s, previous_steering_rad, side,
                           motion_epoch, sensor_epoch, output_dt_s=None):
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
            # 이전 source/proof 검사를 끝낸 내부 소유 current만 조기 전달한다.
            # control 시작 때 구한 실제 dt도 확인해 후속 clamp가 checked 각도를
            # 바꾸지 않게 한다. dt 생략·nonretained는 기존100ms 정책이다.
            if (cur['reference_s'] > self.last_application
                    and _checked_angle_fits_actual_interval(out, previous_steering_rad,
                        output_dt_s, now_s-self.last_application)):
                return None
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
