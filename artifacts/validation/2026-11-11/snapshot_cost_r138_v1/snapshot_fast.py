"""기록 형식 불변의 내장 수치·불변 MeasurementScan 복사 비용 후보."""
from dataclasses import fields
import math
from numbers import Real
from arena_autonomy.measurement_scan import MeasurementScan


def freeze(value):
    kind=type(value)
    if value is None or kind in (str,bool,int):return value
    if kind is float:
        if math.isfinite(value):return value
        return {'nonfinite':'nan' if math.isnan(value) else ('inf' if value>0 else '-inf')}
    if isinstance(value,(str,bool)):return value
    if isinstance(value,Real):
        if not math.isfinite(value):
            return {'nonfinite':'nan' if math.isnan(value) else ('inf' if value>0 else '-inf')}
        return int(value) if isinstance(value,int) else float(value)
    if isinstance(value,dict):return {str(k):freeze(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [freeze(v) for v in value]
    if hasattr(value,'tolist'):return freeze(value.tolist())
    raise TypeError(f'unsupported trace value: {type(value).__name__}')


def snapshot_call(scan,history,reference_s,epoch,speed,side,steering,dt,verified,rear_half_angle):
    # MeasurementScan은 frozen dataclass이며 저장 필드는 스칼라/불변 tuple이다.
    # asdict의 중간 deepcopy 뒤 freeze의 재복사 대신 최종 저장 복사만 수행한다.
    raw=({'representation':'native_measurements_v1','measurements':{f.name:getattr(scan,f.name) for f in fields(scan)}}
         if isinstance(scan,MeasurementScan) else {
            'sec':scan.header.stamp.sec,'nanosec':scan.header.stamp.nanosec,
            'frame_id':scan.header.frame_id,'ranges':list(scan.ranges),
            **{k:getattr(scan,k) for k in ('angle_min','angle_increment','range_min','range_max','time_increment')}})
    return freeze({'scan':raw,
        'history':{k:{'times':list(getattr(history,k).times),'values':list(getattr(history,k).values)} for k in ('wheels','gyro')},
        'reference_s':reference_s,'epoch':epoch,'speed_mps':speed,'side':side,
        'previous_steering_rad':steering,'dt_s':dt,'timing_verified':verified,
        'rear_half_angle_deg':rear_half_angle})
