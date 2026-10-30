"""관측된 우회 준비 대기의 짧은 연속성. 자유 공간이나 우회 허가를 기억하지 않는다."""
import math

import numpy as np

from arena_autonomy.scene_motion import transport_points


class BoundedPreparationHold:
    """새 스캔에서 확인한 실제 준비 대기만 최대1초 보존한다.

    추가 감속용 상태다. 현재 전방 반환과 모터축/IMU 이동 보정을 계속 요구하며
    기존 G2의 현재 공간 검사나 독립 보호 출력을 대신하지 않는다.
    """
    maximum_age_s = 1.0

    def __init__(self):
        self.clear()

    def clear(self):
        self.anchor = None

    def limit(self, controller, scene, history, target, detail):
        from arena_autonomy.bypass_handoff import front_patch

        anchor = self.anchor
        # 호출자는 현재 전방 조각과 G1 진행 원호를 이미 확인했다.
        if anchor is None or detail.get('reason') != 'two_wall_straight_reference_unavailable':
            return target, detail
        now = scene.t_ref_s
        key = (scene.source_frame_id, scene.source_epoch, scene.source_first_s)
        if (key[:2] != anchor['key'][:2] or key[2] < anchor['key'][2]
                or not 0 <= now - anchor['time'] <= self.maximum_age_s
                or not 0 <= now - key[2] < .30 or controller.monitor.motion_evidence()):
            self.clear()
            return target, detail
        patch, _ = front_patch(scene, controller.execution.front_patch_policy)
        if patch is None:
            self.clear()
            return target, detail
        try:
            projected = transport_points(anchor['patch'], history, anchor['time'], now)
            displacement = float(np.linalg.norm(np.median(projected, axis=0) - np.median(patch, axis=0)))
        except ValueError:
            self.clear()
            return target, detail
        if not math.isfinite(displacement) or displacement > controller.execution.maximum_patch_displacement_m:
            self.clear()
            return target, detail
        return 0., dict(detail, applied=target > 0, reason='bounded_preparation_hold_candidate',
            original_reason=detail['reason'], hold_age_s=now - anchor['time'], maximum_age_s=self.maximum_age_s,
            compensated_patch_displacement_m=displacement, prior_source_first_s=anchor['key'][2],
            current_front_evidence=True, free_space_assumed=False, bypass_permission=False)

    def observe(self, controller, scene, output):
        from arena_autonomy.bypass_handoff import front_patch

        detail = output[2]
        if controller.mode != 'FOLLOW' or detail.get('reason') in (
                'handoff_reverse_motion_hold', 'handoff_input_fault',
                'handoff_invalid_input', 'handoff_source_reset'):
            self.clear()
            return
        preparation = detail.get('preparation') or {}
        entry = detail.get('entry_space') or {}
        if output[0] == 0. and preparation.get('applied') is True and entry.get('hold') is True:
            key = (scene.source_frame_id, scene.source_epoch, scene.source_first_s)
            patch, _ = front_patch(scene, controller.execution.front_patch_policy)
            # 같은 스캔을 재사용하는 호출은 유지 시간을 연장하지 않는다.
            if patch is not None and (self.anchor is None or key != self.anchor['key']):
                self.anchor = dict(key=key, time=scene.t_ref_s, patch=patch.copy())
        if preparation.get('reason') in (
                'patch_motion_observed', 'no_current_front_patch', 'patch_outside_g1_swept_arc'):
            self.clear()
