"""추월 준비의 운동학 비교. 실물 가속 보증이나 출력 허가 함수가 아니다."""
import math


def estimate_closure(initial_speed, capped_speed, lead_speed, relative_gain,
                     return_distance, assumed_acceleration=1.0):
    values = (initial_speed,capped_speed,lead_speed,relative_gain,
              return_distance,assumed_acceleration)
    if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in values):
        raise ValueError('finite numeric planning inputs required')
    if (min(initial_speed,lead_speed,return_distance)<0 or relative_gain<=0
            or assumed_acceleration<=0 or capped_speed<=0 or initial_speed>capped_speed):
        raise ValueError('invalid planning envelope')
    if capped_speed<=lead_speed:
        return {'possible_at_assumed_pace':False,'reason':'no_relative_speed_advantage'}
    a=assumed_acceleration
    ramp=(capped_speed-initial_speed)/a
    b=initial_speed-lead_speed
    root=(-b+math.sqrt(b*b+2*a*relative_gain))/a
    if root<=ramp:
        time=root
    else:
        gained=b*ramp+.5*a*ramp*ramp
        time=ramp+(relative_gain-gained)/(capped_speed-lead_speed)
    return dict(possible_at_assumed_pace=True,pass_time_s=time,
                forward_distance_m=relative_gain+lead_speed*time,
                including_return_distance_m=relative_gain+lead_speed*time+return_distance,
                including_return_time_s=time+return_distance/capped_speed,
                assumed_acceleration_mps2=a,future_speed_guaranteed=False)


if __name__=='__main__':
    # 독립적인 작은 시간 간격 적분으로 가속/정속 해를 대조한다. 실제 주행 검증은 아니다.
    for inputs in ((.1,1.1,.7,1.4,1.2),(.0,1.1,.2,.01,1.2),(.65,.65,.3,1.3,1.2)):
        result=estimate_closure(*inputs)
        t=0.;dt=.00005;gain=0.
        while gain<inputs[3]:
            gain+=(min(inputs[0]+t+.5*dt,inputs[1])-inputs[2])*dt
            t+=dt
        assert abs(t-result['pass_time_s'])<=2*dt,(inputs,t,result)
    assert not estimate_closure(.2,.9,.9,1.,1.2)['possible_at_assumed_pace']
    print('3 independent numerical integrations and equal-speed rejection passed')
