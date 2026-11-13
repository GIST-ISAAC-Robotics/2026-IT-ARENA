"""MCU↔호스트 시각의 조건부 구간 추정. ROS/통신/자동 교정과 미연결.

직렬화된 왕복 기록의 네 취득 시각과 명시한 주파수/타임스탬프 오차 안에서
가능한 affine 시계 변환을 교집합으로 구한다. 왕복 지연을 반으로 나눠
편도 지연이라고 가정하지 않는다. 단조 호스트 시각은 ROS 시각과도 별개다.
"""
from dataclasses import dataclass
from fractions import Fraction
import math


def integer(value,name,maximum=2**63-1):
    if type(value) is not int or not 0<=value<=maximum:
        raise ValueError(name+': nonnegative bounded integer required')
    return value


@dataclass(frozen=True, slots=True)
class ClockExchange:
    boot: str
    sequence: int
    host_send_ns: int
    mcu_receive_us: int
    mcu_send_us: int
    host_receive_ns: int

    def __post_init__(self):
        if not isinstance(self.boot,str) or not self.boot.strip() or len(self.boot)>128:
            raise ValueError('nonempty boot ID required')
        for name in ('sequence','host_send_ns','mcu_receive_us','mcu_send_us','host_receive_ns'):
            integer(getattr(self,name),name)
        if self.host_receive_ns<self.host_send_ns or self.mcu_send_us<self.mcu_receive_us:
            raise ValueError('exchange clock/order reversed')


@dataclass(frozen=True, slots=True)
class ClockAssumptions:
    max_rate_error_ppm: int
    host_timestamp_error_ns: int
    mcu_timestamp_error_us: int
    max_window_us: int
    max_extrapolation_us: int

    def __post_init__(self):
        integer(self.max_rate_error_ppm,'rate bound',100_000)
        integer(self.host_timestamp_error_ns,'host timestamp error',1_000_000_000)
        integer(self.mcu_timestamp_error_us,'MCU timestamp error',1_000_000)
        integer(self.max_window_us,'window',60_000_000)
        integer(self.max_extrapolation_us,'extrapolation',60_000_000)
        if self.max_window_us==0:
            raise ValueError('positive fit window required')


@dataclass(frozen=True, slots=True)
class MappedInterval:
    boot: str
    capture_us: int
    host_lower_ns: int
    host_upper_ns: int
    width_ns: int
    extrapolation_us: int
    ready_to_drive: bool = False
    ros_stamp_verified: bool = False
    clock_mapping_applied: bool = False


def clip_polygon(points,a,b,c):
    """a*rate+b*offset <= c. Fraction으로 큰 단조 시각의 정밀도를 유지한다."""
    result=[]
    if not points:
        return ()
    for first,last in zip(points,points[1:]+points[:1]):
        f=a*first[0]+b*first[1]-c
        g=a*last[0]+b*last[1]-c
        if f<=0:
            result.append(first)
        if (f<=0)!=(g<=0):
            ratio=f/(f-g)
            result.append(tuple(first[i]+ratio*(last[i]-first[i]) for i in (0,1)))
    # 선분/점 형태의 0 ppm 또는 정확한 timestamp 조건도 유지한다.
    return tuple(dict.fromkeys(result))


@dataclass(frozen=True, slots=True)
class ClockFit:
    boot: str
    mcu_anchor_us: int
    host_anchor_ns: int
    observed_first_us: int
    observed_last_us: int
    assumptions: ClockAssumptions
    vertices: tuple
    exchanges: int
    sequence_gaps: int

    def map_interval(self,capture_us,*,boot):
        integer(capture_us,'capture_us')
        if boot!=self.boot:
            raise ValueError('cannot map across MCU boot')
        extrapolation=max(self.observed_first_us-capture_us,capture_us-self.observed_last_us,0)
        if extrapolation>self.assumptions.max_extrapolation_us:
            raise ValueError('mapping extrapolation exceeded')
        x=(capture_us-self.mcu_anchor_us)*1000
        error=self.assumptions.mcu_timestamp_error_us*1000
        # 현재 표본의 MCU timestamp 오차도 포함한다. 보정 계수 midpoint를 채택하지 않는다.
        lower=min(rate*(x-error)+offset for rate,offset in self.vertices)+self.host_anchor_ns
        upper=max(rate*(x+error)+offset for rate,offset in self.vertices)+self.host_anchor_ns
        lo,hi=math.floor(lower),math.ceil(upper)
        return MappedInterval(self.boot,capture_us,lo,hi,hi-lo,extrapolation)


def fit_clock(exchanges,assumptions):
    if not isinstance(assumptions,ClockAssumptions):
        raise ValueError('explicit ClockAssumptions required')
    records=tuple(exchanges)
    if not 1<=len(records)<=64 or any(not isinstance(r,ClockExchange) for r in records):
        raise ValueError('one through 64 typed exchanges required')
    first=records[0]
    gaps=0
    for record in records:
        if record.boot!=first.boot:
            raise ValueError('mixed MCU boot epochs')
    for previous,current in zip(records,records[1:]):
        if current.sequence<=previous.sequence:
            raise ValueError('duplicate or reversed exchange sequence')
        if current.host_send_ns<previous.host_receive_ns or current.mcu_receive_us<previous.mcu_send_us:
            raise ValueError('overlapping or reversed serialized exchanges')
        gaps+=current.sequence-previous.sequence-1
    last=records[-1]
    if last.mcu_send_us-first.mcu_receive_us>assumptions.max_window_us:
        raise ValueError('fit window exceeded')
    delta=Fraction(assumptions.max_rate_error_ppm,1_000_000)
    low,high=1-delta,1+delta
    host_error=assumptions.host_timestamp_error_ns
    mcu_error=assumptions.mcu_timestamp_error_us*1000
    lower=-host_error-high*mcu_error
    upper=last.host_receive_ns-first.host_send_ns+host_error+high*mcu_error
    polygon=((low,lower),(high,lower),(high,upper),(low,upper))
    for record in records:
        receive_x=(record.mcu_receive_us-first.mcu_receive_us)*1000
        send_x=(record.mcu_send_us-first.mcu_receive_us)*1000
        host_send=record.host_send_ns-first.host_send_ns
        host_receive=record.host_receive_ns-first.host_send_ns
        # h_send <= mapped(m_receive), mapped(m_send) <= h_receive.
        # 기록 시각의 명시 오차를 두 부등식 바깥 방향으로 확장한다.
        polygon=clip_polygon(polygon,-(receive_x+mcu_error),-1,-host_send+host_error)
        polygon=clip_polygon(polygon,send_x-mcu_error,1,host_receive+host_error)
        if not polygon:
            raise ValueError('inconsistent clock evidence or declared assumptions')
    return ClockFit(first.boot,first.mcu_receive_us,first.host_send_ns,
        first.mcu_receive_us,last.mcu_send_us,assumptions,polygon,len(records),gaps)
