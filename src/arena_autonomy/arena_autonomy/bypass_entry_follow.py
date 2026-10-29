"""G2의 명시적 저속 시뮬레이션 실행 어댑터. 자동 경기 우회 선택기가 아니다."""
from dataclasses import dataclass
from arena_autonomy.bypass_entry_trial import EntryConfig,EntryGuide
from arena_autonomy.local_follow import FollowConfig


@dataclass(frozen=True)
class BypassExecutionConfig:
    stationary_outer_boundary: bool = True
    wall_reanchor: bool = True
    tracked_wall_fit: bool = True
    rejoin_enabled: bool = True
    wide_entry: bool = True
    prefer_earlier_transition: bool = False

    def __post_init__(self):
        if any(type(value) is not bool for value in vars(self).values()):
            raise ValueError('explicit boolean G2 execution options required')
        if self.tracked_wall_fit and not self.wall_reanchor:
            raise ValueError('tracked wall fit requires reanchor')
        if self.wall_reanchor and not self.stationary_outer_boundary:
            raise ValueError('reanchor requires explicit stationary outer boundary assumption')


class BypassEntryFollow:
    def __init__(self,config=None,execution=None,*,allow_simulation_trial=False):
        if allow_simulation_trial is not True:
            raise ValueError('G2 execution requires explicit simulation trial')
        self.config=config or FollowConfig()
        self.execution=execution or BypassExecutionConfig()
        offsets=(0.,-.16,.16,-.20,.20,-.24,.24)+((-.27,.27) if self.execution.wide_entry else ())
        self.guide=EntryGuide(self.config,EntryConfig(offsets_m=offsets,
            prefer_earlier_transition=self.execution.prefer_earlier_transition),
            stationary_outer_boundary=self.execution.stationary_outer_boundary,
            wall_reanchor=self.execution.wall_reanchor,tracked_wall_fit=self.execution.tracked_wall_fit,
            rejoin_enabled=self.execution.rejoin_enabled)

    def reset(self):
        self.guide.reset()

    def command(self,scene,history,speed,side,previous_steering,dt,*,raw_scan=None):
        result=self.guide.evaluate(scene,history,speed,previous_steering,dt)
        chosen=result['selected']
        target,steering=(0.,previous_steering) if chosen is None else (chosen['speed_mps'],chosen['steering_rad'])
        detail={**result,'mode':'g2_entry_trial','behavior':'BYPASS' if target>0 else 'HOLD',
            'passing_enabled':True,'simulation_trial_only':True,'automatic_race_mode_selection':False,
            'output_requires_independent_guard':True,'source_first_s':scene.source_first_s,
            'reference_stamp_s':scene.t_ref_s,'source_epoch':scene.source_epoch}
        return target,steering,detail
