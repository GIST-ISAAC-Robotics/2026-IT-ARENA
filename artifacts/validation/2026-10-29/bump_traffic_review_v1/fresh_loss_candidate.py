"""미연결 운용 후보: 같은 스캔의 시간 증가로 생긴 가능성은 다음 입력까지 대기.

현재 가까운 반환과 새 스캔에서 확인된 소실은 기존 래치를 유지한다.
과거 객체의 신원을 장기 추적하지 않으므로, 처음 멀리서 소실된 물체가
계속 미관측으로 접근하는 위험을 해소한 정책이 아니다. unknown은 그대로다.
"""
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy


class FreshLossCandidate(RecoveryPolicy):
    def reset(self):
        super().reset()
        self.assessment_fresh=False
        self.same_scan_wait=None

    def evaluate(self,scan,*args,**kwargs):
        source=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
        self.assessment_fresh=source!=self.last_source
        self.same_scan_wait=None
        result=super().evaluate(scan,*args,**kwargs)
        if self.same_scan_wait is not None and not self.latched and result.get('allowed'):
            return {**result,'allowed':False,'reason':'same_scan_missing_wait','speed_cap_mps':0.,
                'same_scan_wait':self.same_scan_wait,'unknown_filled':False,
                'residual_hidden_object_risk':True}
        return result

    def _latch(self,reason,now):
        if (reason=='previous_point_in_missing_sweep' and not self.assessment_fresh
                and not self.latched):
            evidence=self.latch_evidence or {}
            self.same_scan_wait=dict(possible_pairs=evidence.get('pairs'),
                source_first_s=self.last_source,
                reference_s=now,
                fresh_measurement=False,permanent_latch_created=False,
                scope='wait_for_new_scan_not_old_object_absence_proof')
            self.latch_evidence=None
            # 현재 재관측/표면 대응으로 제거되고도 남은 위협에만 적용한다.
            # 이 호출의 정지 판정은 evaluate에서 적용한다. 원시 관측은 보존한다.
            return
        return super()._latch(reason,now)

    def _capture_latched_threat(self,*args,**kwargs):
        if self.same_scan_wait is not None and not self.latched:return
        return super()._capture_latched_threat(*args,**kwargs)
