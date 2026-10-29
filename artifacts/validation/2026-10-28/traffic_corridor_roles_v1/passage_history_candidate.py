"""미연결 후보: 경로 재획득 대기와 이미 확인한 통과 이력을 분리한다.

대기 중 명령/통과 상태를 그대로 유지한다. 새 경로의 fresh 스캔에서만 과거
이력을 기존 retained-passage 검사에 전달한다. 현재 위협·영역·관측·속도/STOP
검사는 원래 선택기가 수행한다. 이력은 현재 자유 공간 증명이 아니다.
"""
import math
from arena_autonomy.pass_release_trial import PassReleaseSelector as OriginalSelector, NORMAL_BASELINE


class SuspendedPassageCandidate(OriginalSelector):
    def reset(self):
        self.suspended_passage=set()
        self.passage_preference=None
        super().reset()

    def invalidate(self,reason):
        self.suspended_passage.clear()
        self.passage_preference=None
        return super().invalidate(reason)

    def _select(self,scene,baseline,parallel,speed,previous,dt,*,raw_scan=None):
        if not self.follow_config.traffic_recovery_trial:
            return super()._select(scene,baseline,parallel,speed,previous,dt,raw_scan=raw_scan)
        detail=baseline[2]
        preferred=detail.get('preferred_wall')
        if preferred not in ('left','right'):
            self.suspended_passage.clear()
        elif self.passage_preference is not None and preferred!=self.passage_preference:
            self.suspended_passage.clear()
        self.passage_preference=preferred
        reason=detail.get('reason')
        if reason not in NORMAL_BASELINE and reason!='follow_boundary_unconfirmed':
            self.suspended_passage.clear()
        prior_passed={n for n,s in self.sides.items() if s['passed'] and not s['present']}
        source=scene.source_first_s
        fresh=(source is not None and math.isfinite(source) and
               (self.last_source is None or source>self.last_source))
        path=detail.get('path_points')
        restored=[]
        if (fresh and reason in NORMAL_BASELINE and path is not None and len(path)>=2
                and preferred in ('left','right')):
            for name in sorted(self.suspended_passage):
                if self.sides[name]['seen'] and not self.sides[name]['present']:
                    self.sides[name]['passed']=True
                    restored.append(name)
            self.suspended_passage.clear()
        output=super()._select(scene,baseline,parallel,speed,previous,dt,raw_scan=raw_scan)
        # 이 후보가 이력을 보관하는 유일한 무경로 사유. 고장/역행/입력 STOP으로
        # 인해 반환된 hold_fault를 경로 대기로 취급하지 않는다.
        if (reason=='follow_boundary_unconfirmed' and output[0]==0.
                and output[2].get('selector_reason')=='baseline_not_normal'
                and preferred in ('left','right')):
            self.suspended_passage.update(prior_passed)
        elif output[2].get('selector_choice')=='hold_fault':
            self.suspended_passage.clear()
        # 같은 fresh 스캔에서 새 위협이면 원래 select가 passed를 즉시 취소한다.
        self.suspended_passage.difference_update(n for n,s in self.sides.items() if s['present'])
        return output[0],output[1],{**output[2],
            'suspended_passage_history':sorted(self.suspended_passage),
            'restored_passage_history':restored,
            'restored_history_is_current_free_space':False}
