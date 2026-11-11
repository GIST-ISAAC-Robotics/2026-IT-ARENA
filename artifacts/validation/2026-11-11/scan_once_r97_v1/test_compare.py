import importlib.util
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('scan_once_compare',Path(__file__).with_name('compare.py'))
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('dt',[.02,.1])
def test_same_scan_ticks_skipped_but_all_new_scans_retained(dt):
    c=module.OncePerScan({1.:(0,.9),1.02:(0,.9),1.10:(0,1.),1.12:(0,1.),1.2:(0,1.1)},dt)
    # cost exceeds scan period, intentionally ignored for this diagnostic selector.
    assert c.offer(1.,.18,.02)==pytest.approx((dt,1.18))
    assert c.offer(1.02,.18,.02) is None
    assert c.offer(1.10,.18,.02)==pytest.approx((dt,1.28))
    assert c.offer(1.12,.18,.02) is None
    assert c.offer(1.2,.18,.02)==pytest.approx((dt,1.38))


def test_reset_accepts_same_source_again_and_time_rewind_remains_explicit():
    c=module.OncePerScan({1.:(0,.9),.5:(1,.4)},.1)
    assert c.offer(1.,.05,.02)
    with pytest.raises(ValueError,match='time reversed'):
        c.offer(.5,.05,.02)
    c.reset()
    assert c.offer(.5,.05,.02)
    c.reset()
    assert c.offer(.5,.05,.02)
