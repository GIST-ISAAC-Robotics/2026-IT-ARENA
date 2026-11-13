"""명시 native 선택·ABI buffer 경계·공통 함수 동등성. 컴파일 모듈이 있어야 실행."""
import numpy as np
import pytest

from arena_autonomy import numeric_kernels as backend
extension = pytest.importorskip('arena_autonomy._numeric_kernels', reason='명시 native 후보의 컴파일 모듈 필요')
from arena_autonomy import observation_support,recovery_geometry
from test_local_follow import history,scan
from arena_autonomy.scene_motion import motion_scene


@pytest.fixture(autouse=True)
def reset():
    backend.configure('python')
    yield
    backend.configure('python')


def test_default_does_not_enable_native():
    assert backend._backend is None
    assert backend.polygon(None,None) is None
    assert backend.masked(None,None,None,None,None,None,None,None) is None
    with pytest.raises(ValueError):backend.configure('automatic')


@pytest.mark.parametrize('seed',[19,73])
def test_common_depth_and_polygon_match_after_explicit_selection(seed):
    rng=np.random.default_rng(seed)
    raw=scan(.5,rng.uniform(.06,12.,500).tolist())
    scene=motion_scene(raw,history(.9,-1.3),.635,timing_verified=True,rear_half_angle_deg=30.)
    q=rng.uniform([-.3,-.7],[2.,.7],(600,2));pad=rng.uniform(0.,.2,500)
    expected=observation_support.depth_support(scene,q,pad)
    polygon=rng.normal(size=(7,2));distance=recovery_geometry.polygon_distance(q,polygon)
    backend.configure('native_v1')
    np.testing.assert_array_equal(observation_support.depth_support(scene,q,pad),expected)
    np.testing.assert_allclose(recovery_geometry.polygon_distance(q,polygon),distance,rtol=1e-14,atol=1e-14)


@pytest.mark.parametrize('invalid',['type','length','readonly','noncontiguous','oversize'])
def test_extension_rejects_invalid_buffer_before_read(invalid):
    q=np.zeros((2,2));p=np.zeros((4,2));out=np.empty(2)
    if invalid=='type':q=q.astype(np.float32)
    elif invalid=='length':out=np.empty(1)
    elif invalid=='readonly':out.flags.writeable=False
    elif invalid=='noncontiguous':q=np.zeros((2,4))[:,::2]
    with pytest.raises((ValueError,BufferError)):
        extension.polygon(1000001 if invalid=='oversize' else 2,4,1,q,p,out)


def test_depth_rejects_negative_index_and_keeps_input():
    q=np.zeros((1,2));chosen=np.array([[-1]],dtype=np.int64)
    o=np.zeros((1,2));d=np.array([[1.,0.]]);i=np.array([0],dtype=np.int64)
    valid=np.ones(1,dtype=np.uint8);depth=np.ones(1);padding=np.zeros(1);out=np.empty(1,dtype=np.uint8)
    with pytest.raises(ValueError,match='index outside'):extension.depth(1,1,1,-1,q,chosen,o,d,i,valid,depth,padding,out)
    assert chosen[0,0]==-1


def test_native_child_plan_and_current_result_match_python(monkeypatch):
    import test_recheck_process as protocol
    header,calls=protocol.inputs()
    header['numeric_execution']='native_v1'
    monkeypatch.setattr(protocol,'inputs',lambda:(header,calls))
    protocol.test_input_owned_output_identical_and_single_use()


@pytest.mark.parametrize('invalid',['type','length','readonly','noncontiguous','oversize','negative_index'])
def test_masked_kernel_rejects_invalid_buffers_or_index(invalid):
    q=np.zeros((2,2));chosen=np.array([[0],[0]],dtype=np.int64)
    origins=np.zeros((1,2));directions=np.array([[1.,0.]])
    indices=np.array([0],dtype=np.int64);status=np.array([2],dtype=np.uint8)
    depth=np.ones(1);padding=np.zeros(1);out=np.empty(2,dtype=np.uint8)
    if invalid=='type':q=q.astype(np.float32)
    elif invalid=='length':out=np.empty(1,dtype=np.uint8)
    elif invalid=='readonly':out.flags.writeable=False
    elif invalid=='noncontiguous':q=np.zeros((2,4))[:,::2]
    elif invalid=='negative_index':chosen[0,0]=-1
    with pytest.raises((ValueError,BufferError)):
        extension.masked(1000001 if invalid=='oversize' else 2,1,1,q,chosen,origins,directions,indices,status,depth,padding,out)


def test_masked_kernel_preserves_absence_and_first_tie():
    backend.configure('native_v1')
    q=np.array([[.2,0.]])
    origins=np.zeros((2,2));directions=np.array([[1.,0.],[1.,0.]])
    indices=np.array([0,1]);depth=np.ones(2);padding=np.zeros(2)
    # 같은 각도이면 먼저 오는 광선을 유지: 첫 반환만 선택되면 masked가 아님.
    for status,wanted in [(np.array([1,2]),False),(np.array([2,1]),True),(np.array([0,2]),False)]:
        assert backend.masked(q,np.array([[0,1]]),origins,directions,indices,status,depth,padding)[0]==wanted
        assert not backend.masked(q,np.array([[2,2]]),origins,directions,indices,status,depth,padding)[0]
