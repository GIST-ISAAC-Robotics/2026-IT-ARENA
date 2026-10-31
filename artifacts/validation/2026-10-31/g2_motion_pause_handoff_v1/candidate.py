"""비연결 비교: 명시 signed대기를 진짜 입력고장과 구별하되 출력정지는 유지한다."""
import importlib.util
import inspect
import math
from pathlib import Path
import textwrap
from unittest.mock import patch
import arena_autonomy.bypass_handoff as handoff
import arena_autonomy.bypass_handoff_corridor_trial as corridor

SOURCE=Path(__file__).resolve().parent.parent/'g2_overspeed_hold_v1/candidate.py'
spec=importlib.util.spec_from_file_location('overspeed_hold_candidate_v1',SOURCE)
overspeed=importlib.util.module_from_spec(spec);spec.loader.exec_module(overspeed)


def signed_bypass_pause(owner,normal,speed):
    """G1의 정확한 음수대기만 G2의 이미 구현된 signed대기로 넘긴다."""
    if (owner.mode not in ('BYPASS','REACQUIRE') or owner.execution.entry_policy!='corridor_local_v1'
            or owner.bypass.guide.path is None or isinstance(speed,bool)
            or not isinstance(speed,(int,float)) or not math.isfinite(speed)
            or not -owner.config.max_speed_mps<=speed<0):
        return False
    detail=normal[2]
    measured=detail.get('measured_speed_mps')
    return (normal[0]==0 and detail.get('reason')=='pass_release_reverse_motion_hold'
        and detail.get('selector_choice')=='hold_fault' and detail.get('selector_reason')=='reverse_motion_hold'
        and detail.get('reverse_motion') is True and detail.get('input_error') is None
        and isinstance(measured,(int,float)) and not isinstance(measured,bool)
        and math.isfinite(measured) and abs(measured-speed)<=1e-12)


# 저장 입력에서 한 분류만 비교한다. 나머지 command 본문·센서 유효성·STOP·출력은
# 현재 소스 그대로이며, production 모듈/설치 파일은 수정하지 않는다.
original=textwrap.dedent(inspect.getsource(handoff.BypassHandoffFollow._command))
needle="fault = normal[2].get('selector_choice') == 'hold_fault'"
assert original.count(needle)==1
modified=original.replace(needle,needle+' and not signed_bypass_pause(self,normal,speed)')
namespace=dict(vars(handoff),signed_bypass_pause=signed_bypass_pause)
exec(compile(modified,'offline_signed_handoff_command','exec'),namespace)


class CandidateFollow(handoff.BypassHandoffFollow):
    def __init__(self,*args,**kwargs):
        with patch.object(corridor,'ForecastGuide',overspeed.OverspeedHoldGuide):
            super().__init__(*args,**kwargs)

    _command=namespace['_command']
