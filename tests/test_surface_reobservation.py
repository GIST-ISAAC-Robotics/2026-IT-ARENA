"""관측 네 점 재대응: 표면과 불연속/가림/얇은 표적을 구별하는 제한 반례."""
import numpy as np
import pytest

from arena_autonomy.surface_reobservation import supported_surface
from arena_autonomy.partial_observation import short_observed_surface


def patch():
    # run2의 첫 소실 주변 센서 반환. 정답 지도/차체 자세가 아니다.
    return np.array([[.03243424791,.54898857224], [.02602671068,.48121487393],
                     [.02043058775,.45834502092], [.01498203937,.44856028322]])


@pytest.mark.parametrize('sign', [1.,-1.])
def test_four_consecutive_returns_support_sparse_oblique_patch(sign):
    p=patch();p[:,1]*=sign
    old=np.array([.03237031136,.51284943630*sign])
    assert short_observed_surface(old,p,np.arange(377,381)) is None
    witness=supported_surface(old,p,np.arange(377,381))
    assert witness is not None and witness['support_kind']=='four_consecutive_observed_returns'
    assert not witness['unknown_filled'] and witness['normal_distance_m'] < .01
    np.testing.assert_array_equal(patch()*[1,sign],p)


def test_legacy_short_patch_is_unchanged():
    p=np.array([[.2,.4],[.245,.4],[.29,.4]])
    assert supported_surface([.222,.405],p,[100,101,102])==short_observed_surface([.222,.405],p,[100,101,102])


@pytest.mark.parametrize('fault', ['missing','duplicate','reverse_ids','float_ids','three','too_long',
    'thin_foreground','step','normal_distance','outside','far','nan'])
def test_sparse_extension_does_not_join_missing_or_unrelated_returns(fault):
    p=patch();point=np.array([.03237031136,.51284943630]);ids=np.arange(377,381)
    if fault=='missing': ids[2:]+=1
    if fault=='duplicate':ids[2]=ids[1]
    if fault=='reverse_ids':ids=ids[::-1]
    if fault=='float_ids':ids=ids.astype(float)
    if fault=='three':p=p[:3];ids=ids[:3]
    if fault=='too_long':p[0,1]+=.06
    if fault=='thin_foreground':p[0,0]-=.06
    if fault=='step':p[2:,0]+=.06
    if fault=='normal_distance':point[0]+=.03
    if fault=='outside':point=p[0]+[0,.025]
    if fault=='far':point+=1
    if fault=='nan':p[0,0]=np.nan
    assert supported_surface(point,p,ids) is None
