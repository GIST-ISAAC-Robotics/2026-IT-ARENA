"""비연결 후보: 복귀 조향 포화 때 현재 벽에서 관측된 길이만 다시 제안한다."""
import inspect,math,textwrap
import numpy as np
from unittest.mock import patch
import arena_autonomy.bypass_entry_trial as entry_module
from arena_autonomy.bypass_handoff import BypassHandoffFollow
from arena_autonomy.local_follow import pursue_path


def refresh_rejoin_path(path,fit,config,entry,phase,fresh,confirmations,speed,age):
    detail=dict(applied=False,reason='not_fresh_saturated_rejoin',proposal_is_permission=False)
    if phase!='rejoin' or not fresh or confirmations<2:
        return path,detail
    if (not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
                for v in (*fit,speed,age)) or speed<0 or speed>config.max_speed_mps or age<0):
        return path,dict(detail,reason='invalid_measurement')
    try:
        _,old_steering,_=pursue_path(path,speed,config,age)
        if abs(old_steering)<config.max_steering_rad-1e-12:
            return path,detail
        transition=entry.transitions_m[-1]
        proposed=entry_module.entry_path(fit,0.,config,entry,transition)
        if proposed[-1,0]-proposed[0,0]<transition:
            return path,dict(detail,reason='current_wall_too_short_for_return')
        _,new_steering,_=pursue_path(proposed,speed,config,age)
        if abs(new_steering)>=abs(old_steering)-1e-12:
            return path,dict(detail,reason='new_proposal_not_less_saturated')
    except (ValueError,TypeError,IndexError,entry_module.MotionUnavailable):
        return path,dict(detail,reason='proposal_unavailable')
    return proposed,dict(detail,applied=True,reason='current_wall_return_proposal',
        old_pursuit_steering_rad=old_steering,new_pursuit_steering_rad=new_steering,
        transition_m=transition,observed_x_m=list(fit[2:]),truth_or_map_used=False)


original=textwrap.dedent(inspect.getsource(entry_module.EntryGuide.evaluate))
needle='    if np.any(np.diff(path[:,0])<=0):'
insertion=('    path,rejoin_refresh=refresh_rejoin_path(path,fit,self.config,self.entry,\n'
           '        self.phase,fresh,self.wall_confirmations,speed,now-scene.source_first_s)\n')
assert original.count(needle)==1
modified=original.replace(needle,insertion+needle)
tail="'center_confirmations':self.center_confirmations}"
assert modified.count(tail)==1
modified=modified.replace(tail,"'center_confirmations':self.center_confirmations,'rejoin_refresh':rejoin_refresh}")
namespace=dict(vars(entry_module),refresh_rejoin_path=refresh_rejoin_path)
exec(compile(modified,'offline_rejoin_refresh_evaluate','exec'),namespace)


class CandidateFollow(BypassHandoffFollow):
    def command(self,*args,**kwargs):
        self.last_refresh_calls=[]
        def observe(*a):
            result=refresh_rejoin_path(*a)
            if result[1]['applied']:self.last_refresh_calls.append(result[1])
            return result
        with patch.dict(namespace,refresh_rejoin_path=observe),patch.object(entry_module.EntryGuide,'evaluate',namespace['evaluate']):
            return super().command(*args,**kwargs)
