"""r377과 같은 시설 평가용 mesh slab. 원 함수의 의존 이름만 로컬 함수로 연결."""

import struct

import numpy as np

from shapely.geometry import LineString, Polygon

from shapely.ops import polygonize, unary_union



def load(path):
    data = path.read_bytes()
    header = data[:80]
    n = struct.unpack("<I", data[80:84])[0]
    assert len(data) == 84 + 50 * n, (len(data), n)
    rec = np.frombuffer(data[84:], dtype=np.dtype([
        ("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n)
    return header, rec["n"].astype(float), rec["v"].astype(float), rec["a"]



def slice_xy(tris, axis, level):
    """Return segments where plane coord[axis]==level cuts triangles."""
    segs = []
    for t in tris:
        d = t[:, axis] - level
        if d.min() > 0 or d.max() < 0:
            continue
        pts = []
        for i in range(3):
            a, b = t[i], t[(i + 1) % 3]
            da, db = d[i], d[(i + 1) % 3]
            if da == 0:
                pts.append(a)
            if da * db < 0:
                s = da / (da - db)
                pts.append(a + s * (b - a))
        if len(pts) >= 2:
            segs.append(np.array(pts[:2]))
    return segs



def section(tris,height):
    lines=slice_xy(tris,2,height)
    segments=np.round(np.asarray(lines)[:,:,:2],9)
    # 삼각망 면을 자른 작은 선분을 연결한다. 빈 U/버팀대 공간을 외접 상자로 채우지 않는다.
    network=unary_union([LineString(s) for s in segments if np.linalg.norm(s[1]-s[0])>1e-9])
    material=[]
    for poly in polygonize(network):
        p=poly.representative_point();crossings=0
        for a,b in segments:
            if (a[1]<=p.y<b[1]) or (b[1]<=p.y<a[1]):
                x=a[0]+(p.y-a[1])/(b[1]-a[1])*(b[0]-a[0])
                crossings+=int(x>p.x)
        if crossings%2:material.append(poly)
    shape=unary_union(material)
    assert not shape.is_empty and shape.is_valid
    return shape



def slab_projection(tri,lo,hi):
    """닫힌 mesh의 높이 구간 투영. 외접상자로 버팀대 빈틈을 채우지 않는다."""
    polygons=[section(tri,lo),section(tri,hi)]
    for face in tri:
        vertices=list(face)
        for z,above in ((lo,True),(hi,False)):
            clipped=[]
            for p,q in zip(vertices,vertices[1:]+vertices[:1]):
                inside=lambda v:v[2]>=z if above else v[2]<=z
                ip,iq=inside(p),inside(q)
                if ip:clipped.append(p)
                if ip!=iq:clipped.append(p+(q-p)*(z-p[2])/(q[2]-p[2]))
            vertices=clipped
            if not vertices:break
        if len(vertices)>=3:
            poly=Polygon([p[:2] for p in vertices])
            if poly.area>1e-14:polygons.append(poly)
    result=unary_union(polygons)
    assert result.is_valid
    return result


