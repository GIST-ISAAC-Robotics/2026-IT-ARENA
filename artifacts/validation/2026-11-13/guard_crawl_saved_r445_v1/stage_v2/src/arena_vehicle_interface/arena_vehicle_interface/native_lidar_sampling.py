"""C1 각도/점수 템플릿의 시뮬 순차 취득. SDK 시간의 실측 재현을 뜻하지 않는다."""
import math


class NativeRevolutionAssembler:
    def __init__(self, patterns_q14, *, rate_hz=10., max_gap_s=.0021):
        if not math.isfinite(rate_hz) or not 8 <= rate_hz <= 12 or not 0 < max_gap_s <= .0021:
            raise ValueError('invalid native simulation rate/gap')
        self.patterns=[]
        for pattern in patterns_q14:
            if not 300 <= len(pattern) <= 800 or any(type(a) is not int or not 0 <= a <= 65535 for a in pattern):
                raise ValueError('invalid C1 sampling template')
            angles=tuple(math.pi-a*math.pi/32768 for a in pattern)
            gaps=[math.remainder(b-a,math.tau) for a,b in zip(angles,angles[1:])]
            if not all(-math.radians(2)<=v<0 for v in gaps) or not 5.5<=abs(sum(gaps))<=7:
                raise ValueError('template is not a clockwise revolution')
            self.patterns.append(angles)
        if not self.patterns: raise ValueError('empty sampling templates')
        self.period=1/rate_hz;self.max_gap=max_gap_s
        self.previous=None;self.start=None;self.sequence=0;self.epoch=0;self.discarded=0
        self.ranges=[];self.angles=[];self.errors=[];self.angular_errors=[]
        self.source_metadata=None

    def feed(self, stamp, ranges, angle_min, angle_increment):
        if (not all(math.isfinite(v) for v in (stamp,angle_min,angle_increment)) or stamp<0
                or len(ranges)<2048 or angle_increment<=0
                or abs(len(ranges)*angle_increment-math.tau)>1e-5):
            raise ValueError('native simulation needs a dense uniform source revolution')
        metadata=(len(ranges),angle_min,angle_increment)
        if self.source_metadata is not None and metadata!=self.source_metadata:
            raise ValueError('instantaneous source geometry changed')
        self.source_metadata=metadata
        if self.previous is None or stamp<self.previous[0] or stamp-self.previous[0]>self.max_gap:
            if self.previous is not None:
                if stamp<self.previous[0]: self.epoch+=1
                else: self.discarded+=1
            self.previous=(stamp,tuple(ranges));self.start=stamp
            self.ranges=[];self.angles=[];self.errors=[];self.angular_errors=[]
            return []
        if stamp==self.previous[0]: return []
        pattern=self.patterns[self.sequence%len(self.patterns)]
        increment=self.period/len(pattern)
        previous_stamp,previous_ranges=self.previous
        while len(self.ranges)<len(pattern) and self.start+len(self.ranges)*increment<stamp-1e-10:
            i=len(self.ranges);target=pattern[i]
            index=round(((target-angle_min)%math.tau)/angle_increment)%len(ranges)
            actual=angle_min+index*angle_increment
            # 거리와 실제 source 광선 방향을 짝으로 유지한다. 표적 각도를 거리만
            # 다른 방향에서 읽은 값에 붙이지 않는다. 오차는 audit에 따로 남긴다.
            self.ranges.append(previous_ranges[index]);self.angles.append(actual)
            self.errors.append(max(0.,self.start+i*increment-previous_stamp))
            self.angular_errors.append(abs(math.remainder(actual-target,math.tau)))
        output=[]
        if len(self.ranges)==len(pattern) and stamp+1e-10>=self.start+self.period:
            output.append(dict(ranges_m=self.ranges,angles_rad=self.angles,
                acquired_at_s=[self.start+i*increment for i in range(len(pattern))],
                source_epoch=self.epoch,template_index=self.sequence%len(self.patterns),
                max_capture_time_error_s=max(self.errors),
                max_source_angle_quantization_rad=max(self.angular_errors)))
            self.start+=self.period;self.sequence+=1
            self.ranges=[];self.angles=[];self.errors=[];self.angular_errors=[]
        self.previous=(stamp,tuple(ranges))
        return output


def encode_measurements(values, *, range_min, range_max, frame_id):
    """NaN은 null, 양/음의 무한대는 명시 태그로 원 구분을 보존한다. 자유 공간을 만들지 않는다."""
    return dict(schema='arena_native_lidar_v1',ranges_m=[float(v) if math.isfinite(v) else ('+inf' if v==math.inf else '-inf' if v==-math.inf else None) for v in values['ranges_m']],
        angles_rad=values['angles_rad'],acquired_at_s=values['acquired_at_s'],
        source_frame_id=frame_id,source_epoch=values['source_epoch'],range_min_m=range_min,range_max_m=range_max,
        time_basis='simulation_acquisition')
