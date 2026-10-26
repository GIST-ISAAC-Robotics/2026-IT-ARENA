import math
import numpy as np
import pytest
from parallel_wall_normal_probe import normal_interval,search_normal


def measured(theta=.027,center=.04,bias=.029,half=.425):
    s=np.repeat([1.,-1.],31)
    x=np.tile(np.linspace(-.1,.8,31),2)
    local=np.column_stack((x,center+s*half))
    c,sn=math.cos(theta),math.sin(theta)
    p=local@np.array([[c,-sn],[sn,c]]).T
    d=p/np.linalg.norm(p,axis=1)[:,None]
    return p+bias*d,d,s


@pytest.mark.parametrize('theta,bias',[(.027,.029),(-.04,-.029),(0.,0.)])
def test_same_normal_explains_both_sides_and_keeps_true_center(theta,bias):
    p,d,s=measured(theta=theta,bias=bias)
    exact=normal_interval(p,d,s,theta)
    assert exact['center_interval_m'][0] <= .04 <= exact['center_interval_m'][1]
    found=search_normal(p,d,s)
    assert found['status']=='compatible' and found['driving_authorized'] is False
    assert abs(found['theta_rad']-theta)<.003


def test_radial_projection_is_not_full_transverse_error_at_grazing_ray():
    p=np.array([[1.,.435],[1.,-.435]])
    d=p/np.linalg.norm(p,axis=1)[:,None]
    radial=normal_interval(p,d,[1,-1],0.)
    disk=normal_interval(p,d,[1,-1],0.,radial=False)
    assert radial['upper_m']-radial['lower_m'] < disk['upper_m']-disk['lower_m']


def test_center_average_cannot_hide_inconsistent_width():
    p,d,s=measured(theta=0.,bias=0.,half=.49)
    result=search_normal(p,d,s)
    assert result['status']=='inconsistent' and result['gap_m']>.05
    assert result['center_interval_m'] is None and not result['driving_authorized']


def test_missing_side_remains_missing():
    p,d,s=measured()
    result=search_normal(p[:31],d[:31],s[:31])
    assert result['status']=='missing_side' and result['center_interval_m'] is None


@pytest.mark.parametrize('bad', ['direction','point','side','angle'])
def test_invalid_input_rejected(bad):
    p,d,s=measured(); kw={}
    if bad=='direction': d[0]*=2
    if bad=='point': p[0,0]=np.nan
    if bad=='side': s[0]=0
    if bad=='angle':kw['theta_lo']=1.;kw['theta_hi']=0.
    with pytest.raises(ValueError):search_normal(p,d,s,**kw)
