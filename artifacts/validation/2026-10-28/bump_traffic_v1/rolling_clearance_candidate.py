"""미연결 후보: 같은 전체 위협의 연속 정합 좌표를 현재 반환으로 확인한다.

정적 배경 가정은 기존 재관측과 같다. 원래1초·두 새 스캔·현재 입력 재검사를
유지하며 물체 신원 추적이나 미관측 공간의 자유 판정으로 사용하지 않는다.
"""
import math
import numpy as np

from arena_autonomy.anchor_clearance_trial import PositiveClearanceTrial, fresh_clearance
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy


class RollingClearanceCandidate(PositiveClearanceTrial):
    def reset(self):
        super().reset()
        self.rolling_anchor = None
        self.rolling_last_source = None
        self.rolling_consecutive = 0

    def bind(self, anchor):
        """처음 캡처한 바로 그 전체 점/배경/시각에만 연결한다."""
        if (not isinstance(anchor, SurfaceAnchorTrial) or self.original is None
                or anchor.anchor is None or anchor.ended
                or anchor.created != self.original[2]
                or not np.array_equal(anchor.retained, self.original[0])
                or not np.array_equal(anchor.anchor[0], self.original[1])):
            raise ValueError('rolling anchor must share the complete original evidence')
        self.rolling_anchor = anchor
        self.rolling_last_source = anchor.last_source

    def observe(self, scene, history, current, ids):
        original = super().observe(scene, history, current, ids)
        if original.get('matched') or self.ended or self.original is None:
            self.rolling_consecutive = 0
            return original
        anchor = self.rolling_anchor
        if anchor is None:
            return original
        now, source = scene.t_ref_s, scene.source_first_s
        if source == self.rolling_last_source:
            return {**original, 'rolling_clearance':dict(reason='same_scan_no_confirmation',
                consecutive_fresh_sources=self.rolling_consecutive)}
        self.rolling_last_source = source
        points = np.asarray(current, dtype=float)
        # anchor.anchor는 적합을 승인했을 때만 현재 점군/시각으로 바뀐다.
        checked = (not anchor.ended and anchor.anchor is not None
            and anchor.created == self.original[2]
            and math.isfinite(now) and 0 <= now-anchor.created <= 1.
            and anchor.anchor[1] == now and anchor.anchor[2] == source
            and anchor.last_reference == now and anchor.last_source == source
            and anchor.retained.shape == self.original[0].shape
            and np.all(np.isfinite(anchor.retained))
            and anchor.anchor[0].shape == points.shape
            and np.array_equal(anchor.anchor[0], points))
        if not checked:
            self.rolling_consecutive = 0
            return {**original, 'rolling_clearance':dict(reason='current_complete_anchor_required',
                consecutive_fresh_sources=0)}
        proof = fresh_clearance(anchor.retained, points, now, scene, history, points, ids)
        self.rolling_consecutive = self.rolling_consecutive+1 if proof['accepted'] else 0
        proof = {**proof, 'consecutive_fresh_sources':self.rolling_consecutive,
                 'original_reference_s':self.original[2], 'current_reference_s':now,
                 'complete_points':len(anchor.retained), 'static_background_assumption':True}
        if self.rolling_consecutive < 2:
            return {**original, 'rolling_clearance':proof}
        return {**proof, 'reason':'rolling_old_locations_supported', 'matched':True,
                'original_clearance':original, 'rolling_clearance':proof}


class RollingRecoveryCandidate(RecoveryPolicy):
    """시험 프로세스에서만 사용. 기존 해제 후 현재 입력 재검사를 그대로 거친다."""
    def reset(self):
        super().reset()
        self.clearance_probe = RollingClearanceCandidate()

    def _capture_latched_threat(self, *args):
        super()._capture_latched_threat(*args)
        self.clearance_probe.bind(self.anchor_probe)
