import math
from types import SimpleNamespace
import numpy as np
import pytest
from arena_autonomy.follow_trace import freeze as original, snapshot_call as old_snapshot
from arena_autonomy.measurement_scan import MeasurementScan
from snapshot_fast import freeze,snapshot_call
from test_local_follow import scan,history


@pytest.mark.parametrize('value',[None,True,False,1,2**80,-0.,1.23,math.nan,math.inf,-math.inf,
    np.float32(1.2),np.float64(math.nan),np.int64(7),np.bool_(True),[1,(2.,math.nan)],
    {1:np.array([1.,math.inf])},np.array([[1,2],[3,4]])])
def test_exact_numeric_and_nonfinite_encoding(value):
    assert freeze(value)==original(value)


@pytest.mark.parametrize('value',[object(),set([1]),complex(1,2)])
def test_unsupported_types_keep_error(value):
    with pytest.raises(TypeError):freeze(value)
    with pytest.raises(TypeError):original(value)


@pytest.mark.parametrize('native',[False,True])
def test_snapshot_keeps_input_contract_and_ownership(native):
    h=history();raw=scan(.5,[4.]*500)
    if native:
        raw=MeasurementScan(tuple([4.,math.inf,math.nan]+[4.]*497),tuple(-math.pi+i*math.tau/500 for i in range(500)),
            tuple(.5+i*.0002 for i in range(500)),time_basis='simulation_acquisition')
    args=(raw,h,.6,0,0.,'left',.2,.02,True,0.)
    saved=snapshot_call(*args)
    assert saved==old_snapshot(*args)
    before=list(h.gyro.values)
    saved['history']['gyro']['values'][0]=12345.
    assert list(h.gyro.values)==before
    if native:saved['scan']['measurements']['ranges_m'][0]=-9.
    else:saved['scan']['ranges'][0]=-9.
    assert raw.ranges[0]==4.
