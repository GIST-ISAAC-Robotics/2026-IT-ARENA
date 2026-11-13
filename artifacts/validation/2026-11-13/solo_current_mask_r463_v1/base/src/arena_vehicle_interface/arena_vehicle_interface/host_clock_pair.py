"""같은 호스트 두 시계를 짧은 읽기 구간으로 대응시킨다. 센서 취득 교정은 아니다."""


def sample_clocks(monotonic_ns, ros_ns, attempts=3):
    if type(attempts) is not int or not 1<=attempts<=5:
        raise ValueError('bounded clock reads required')
    samples=[]
    for _ in range(attempts):
        before=monotonic_ns()
        ros=ros_ns()
        after=monotonic_ns()
        if any(type(t) is not int or t<0 for t in (before,ros,after)) or after<before:
            raise ValueError('invalid clock sampling interval')
        midpoint=before+(after-before)//2
        samples.append(dict(before_monotonic_ns=before,after_monotonic_ns=after,
                            ros_ns=ros,midpoint_monotonic_ns=midpoint,width_ns=after-before))
    index=min(range(len(samples)),key=lambda i:samples[i]['width_ns'])
    chosen=samples[index]
    diagnostic=dict(policy='narrowest_bracket_v1',samples=samples,selected=index,
                    initial_receipt_monotonic_ns=samples[0]['before_monotonic_ns'],
                    read_interval_half_width_ns=(chosen['width_ns']+1)//2,
                    # 시계 해상도/실제 clock step/센서 transport 오차를 포함하지 않는다.
                    sensor_acquisition_accuracy_verified=False,clock_offset_smoothed=False)
    return chosen['midpoint_monotonic_ns'],chosen['ros_ns'],diagnostic
