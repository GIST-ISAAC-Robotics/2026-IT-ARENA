"""묶음 교차와 기존 scalar 교차의 정확 대조. 경계·빈 조각·미세 영역을 포함한다."""
import numpy as np
import pytest
from arena_autonomy.linear_observation import clip_halfplane
from arena_autonomy.observation_support import outside_body
from arena_autonomy.polygon_batch import clip_polygons, exterior_queries


@pytest.mark.parametrize('scale',[1.,1e-4,1e-9])
@pytest.mark.parametrize('sign',[-1,1])
def test_batched_halfplanes_preserve_vertex_order_and_exact_values(scale,sign):
    rng=np.random.default_rng(1026)
    centers=rng.uniform(-.3,.3,(160,2));angles=rng.uniform(-np.pi,np.pi,160)
    polygons=[]
    for c,theta in zip(centers,angles):
        a,b=np.cos(theta),np.sin(theta)
        polygons.append(np.array([[-.1,-.075],[.1,-.075],[.1,.075],[-.1,.075]]) @
                        np.array([[a,b],[-b,a]])*scale+c)
    polygons=np.array(polygons);counts=np.full(len(polygons),4)
    origins=centers+rng.uniform(-.15,.15,(len(polygons),2))*scale
    # 경계 꼭짓점을 직접 지나가는 조건도 포함한다.
    origins[:40]=polygons[:40,0]
    directions=np.column_stack((np.cos(angles),np.sin(angles)))
    got,sizes=clip_polygons(polygons,counts,origins,directions,sign)
    for i,p in enumerate(polygons):
        expected=clip_halfplane(p.tolist(),origins[i],directions[i],sign)
        assert sizes[i]==len(expected)
        np.testing.assert_array_equal(got[i,:sizes[i]],np.array(expected).reshape(-1,2))


def test_exterior_queries_preserve_zero_thickness_and_tiny_intrusions():
    polygons=[]
    for x,y,a in [(0.,0.,0.),(.1,.075,0.),(.1,.075,.01),(-.1,-.075,-.01)]:
        for delta in (0.,1e-11,-1e-11,1e-7,-1e-7,.001,-.001):
            c,s=np.cos(a+delta),np.sin(a+delta)
            polygons.append((np.array([[-.1,-.075],[.1,-.075],[.1,.075],[-.1,.075]]) @
                np.array([[c,s],[-s,c]]) + [x,y+delta]).tolist())
    expected=[];n=0
    for p in polygons:
        for piece in outside_body(p,.2,.15):
            expected.extend(piece);expected.append(np.mean(piece,axis=0));n+=1
    got,m=exterior_queries(np.array(polygons),np.full(len(polygons),4),.2,.15)
    assert m==n
    np.testing.assert_array_equal(np.unique(got,axis=0),np.unique(np.array(expected),axis=0))


def test_empty_batch_and_empty_rows():
    got,n=clip_polygons(np.zeros((0,4,2)),np.array([],dtype=int),(0,0),(1,0),1)
    assert got.shape==(0,0,2) and len(n)==0
    got,n=clip_polygons(np.zeros((2,4,2)),np.array([0,0]),(0,0),(1,0),1)
    assert got.shape==(2,0,2) and list(n)==[0,0]
    q,k=exterior_queries(got,n,.2,.15)
    assert q.shape==(0,2) and k==0
