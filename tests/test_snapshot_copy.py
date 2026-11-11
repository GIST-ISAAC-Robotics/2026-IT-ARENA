"""내장 수치 경로의 기록 계약 및 불변 측정의 최종 저장 사본 소유권."""
import math
import numpy as np
import pytest
from arena_autonomy.follow_trace import freeze,snapshot_call
from arena_autonomy.measurement_scan import MeasurementScan
from test_local_follow import scan,history


@pytest.mark.parametrize('value,expected',[
    (True,True),(2**80,2**80),(np.int64(7),7.),(np.bool_(False),False),
    (math.nan,{'nonfinite':'nan'}),(math.inf,{'nonfinite':'inf'}),
    (-math.inf,{'nonfinite':'-inf'}),(np.float32(math.nan),{'nonfinite':'nan'}),
    ({2:np.array([1.,math.inf])},{'2':[1.,{'nonfinite':'inf'}]})])
def test_numeric_types_nonfinite_and_keys_keep_record_format(value,expected):
    assert freeze(value)==expected


def test_negative_zero_and_large_integer_exception_are_preserved():
    assert math.copysign(1.,freeze(-0.))==-1.
    with pytest.raises(OverflowError):freeze(2**10000)
    with pytest.raises(TypeError):freeze(object())


@pytest.mark.parametrize('native',[False,True])
def test_snapshot_owns_its_arrays_and_keeps_raw_time_and_values(native):
    h=history();raw=scan(.5,[4.]*500)
    if native:
        raw=MeasurementScan(tuple([4.,math.inf,math.nan]+[4.]*497),
            tuple(-math.pi+i*math.tau/500 for i in range(500)),tuple(.5+i*.0002 for i in range(500)),
            time_basis='simulation_acquisition')
    saved=snapshot_call(raw,h,.6,0,-.1,'left',.2,.02,True,0.)
    assert saved['speed_mps']==-.1 and saved['reference_s']==.6
    before=list(h.gyro.values)
    saved['history']['gyro']['values'][0]=12345.
    assert list(h.gyro.values)==before
    if native:
        assert saved['scan']['measurements']['acquired_at_s']==list(raw.acquired_at_s)
        assert saved['scan']['measurements']['ranges_m'][1:3]==[{'nonfinite':'inf'},{'nonfinite':'nan'}]
        saved['scan']['measurements']['ranges_m'][0]=-9.
    else:saved['scan']['ranges'][0]=-9.
    assert raw.ranges[0]==4.
