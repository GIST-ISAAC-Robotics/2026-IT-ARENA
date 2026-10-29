import copy
import math
import pytest
from perturb import CASES, ErrorCase, transform
from arena_autonomy.follow_trace import freeze


def fixture():
    return dict(scan=dict(sec=10, nanosec=500_000_000, time_increment=.0002,
                          angle_min=0., angle_increment=.01, ranges=[.01, .5, 2., math.inf, math.nan]),
                history={'gyro': dict(times=[10., 10.005, 10.010], values=[.4, .42, .43]),
                         'wheels': dict(times=[10., 10.01], values=[.3, .4])},
                reference_s=10.51, speed_mps=.4, rear_half_angle_deg=30.)


def test_baseline_and_original_immutable():
    source=freeze(fixture()); before=copy.deepcopy(source)
    assert freeze(transform(source, CASES[0])) == source
    for case in CASES:
        transform(source, case)
    assert source == before


def test_overlapping_gyro_samples_and_same_scan_keep_errors():
    first=freeze(fixture()); second=copy.deepcopy(first)
    second['history']['gyro']['times']=second['history']['gyro']['times'][1:]
    second['history']['gyro']['values']=second['history']['gyro']['values'][1:]
    second['reference_s'] += .02
    a,b=(transform(x, CASES[-1]) for x in (first,second))
    assert a['history']['gyro']['values'][1:] == b['history']['gyro']['values']
    assert freeze(a['scan']) == freeze(b['scan'])


@pytest.mark.parametrize('scale',[.95,1.,1.05])
def test_last_ray_anchored(scale):
    source=fixture(); result=transform(freeze(source), ErrorCase('clock',ray_time_scale=scale))
    def end(s): return s['sec']+s['nanosec']*1e-9+(len(s['ranges'])-1)*s['time_increment']
    assert abs(end(source['scan'])-end(result['scan'])) < 1e-9
    assert result['scan']['time_increment'] == source['scan']['time_increment']*scale


def test_unknown_not_filled_range_not_clamped():
    source=fixture(); result=transform(freeze(source), CASES[-1])
    assert math.isinf(result['scan']['ranges'][3]) and math.isnan(result['scan']['ranges'][4])
    assert all(abs(a-b)<=.03 for a,b in zip(source['scan']['ranges'][:3], result['scan']['ranges'][:3]))
    assert result['history']['wheels'] == source['history']['wheels']
    assert result['speed_mps'] == source['speed_mps']


@pytest.mark.parametrize('kwargs',[{'gyro_sigma':-1},{'angle_bias':math.nan},{'ray_time_scale':0}])
def test_bad_case_rejected(kwargs):
    with pytest.raises(ValueError): ErrorCase('bad',**kwargs)


def test_changed_angle_reapplies_receiver_mask_without_recovering_old_unknown():
    from types import SimpleNamespace
    from arena_autonomy.local_path import mask_scan
    from arena_autonomy.local_scene import scene_from_scan
    source=fixture()
    scan=source['scan']
    scan.update(angle_min=-math.pi,angle_increment=math.tau/500,ranges=[1.]*500)
    scan['ranges']=mask_scan(SimpleNamespace(**scan),30.).ranges
    changed=transform(freeze(source),CASES[2])
    raw=changed['scan']
    scene=scene_from_scan(raw['ranges'],angle_min=raw['angle_min'],angle_increment=raw['angle_increment'],
                          range_min=.01,range_max=12.,acquired_at_s=[10.5+i*.0002 for i in range(500)],
                          t_ref_s=10.7,rear_half_angle_deg=30.)
    assert [r.source_index for r in scene.rays if r.status=='return']==[i for i,v in enumerate(raw['ranges']) if math.isfinite(v)]
    assert any(math.isfinite(a) and math.isnan(b) for a,b in zip(scan['ranges'],raw['ranges']))
    assert all(math.isnan(b) for a,b in zip(scan['ranges'],raw['ranges']) if math.isnan(a))
