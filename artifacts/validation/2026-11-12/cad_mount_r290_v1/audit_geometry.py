"""STL 수평 단면과 원 B-rep 단면의 독립 광선 대조. 렌더/주행/실물 검증 아님."""
import hashlib
import json
from pathlib import Path
import struct

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

from arena_autonomy.cad_mount import mount
from arena_autonomy.local_path import mask_scan
from arena_autonomy.local_scene import blind_parameter
from arena_autonomy.measurement_scan import MeasurementScan

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
DESC = ROOT/'src/arena_description'


def triangles(path):
    raw = path.read_bytes()
    count, = struct.unpack_from('<I', raw, 80)
    assert len(raw) == 84 + 50*count
    data = np.frombuffer(raw, dtype=np.dtype([('normal', '<f4', 3), ('v', '<f4', (3,3)), ('attribute', '<u2')]), offset=84)
    tri = data['v'].astype(float)
    normal = np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0])
    norm = np.linalg.norm(normal,axis=1)
    assert np.all(norm>1e-13)
    assert np.allclose(normal/norm[:,None], data['normal'], atol=2e-4)
    return tri


def horizontal_segments(tri, height):
    result = []
    for face in tri:
        intersections = []
        for a,b in zip(face,np.roll(face,-1,axis=0)):
            if (a[2] <= height < b[2]) or (b[2] <= height < a[2]):
                intersections.append((a+(height-a[2])/(b[2]-a[2])*(b-a))[:2])
        if len(intersections)==2:
            result.append(intersections)
    return np.asarray(result)


def raycast(segments, origin, angles):
    direction = np.c_[np.cos(angles), np.sin(angles)]
    result = np.full(len(angles), np.inf)
    for a,b in segments:
        edge=b-a; delta=a-origin
        denom=direction[:,0]*edge[1]-direction[:,1]*edge[0]
        with np.errstate(divide='ignore', invalid='ignore'):
            distance=(delta[0]*edge[1]-delta[1]*edge[0])/denom
            fraction=(delta[0]*direction[:,1]-delta[1]*direction[:,0])/denom
        valid=(np.abs(denom)>1e-14)&(distance>0)&(fraction>=0)&(fraction<=1)
        result=np.minimum(result,np.where(valid,distance,np.inf))
    return result


def main():
    source=json.loads((ROOT/'outputs/cad_review_2026_11_12/source.json').read_text())
    original=Path('/mnt/c/'+source['source'][3:].replace('\\','/'))
    assert original.is_file()
    assert hashlib.sha256(original.read_bytes()).hexdigest()==source['sha256']
    angles=np.deg2rad(np.arange(-180.,180.,.02)+.007)
    results=[]
    figure,axes=plt.subplots(1,2,figsize=(12,6),layout='constrained')
    for path in sorted((DESC/'config').glob('b0390_c1_cad_20261112_*.yaml')):
        profile=yaml.safe_load(path.read_text()); lidar=profile['lidar']
        origin=np.array(lidar['xyz_m'][:2]); height=lidar['xyz_m'][2]
        mesh_paths=[(path.parent/p).resolve() for p in lidar['scan_plane_meshes']]
        meshes=[triangles(p) for p in mesh_paths]
        sections=np.concatenate([horizontal_segments(t,height) for t in meshes])
        polygons=[]
        for row in lidar['scan_plane_sections']:
            polygons.extend([np.array(row['polygon_xy_m']),*[np.array(w) for w in row['wire_polygons_xy_m']]])
        brep=np.concatenate([np.stack((p,np.roll(p,-1,axis=0)),axis=1) for p in polygons])
        mesh_ranges=raycast(sections,origin,angles); brep_ranges=raycast(brep,origin,angles)
        hit=np.isfinite(mesh_ranges); expected=np.isfinite(brep_ranges)
        assert np.array_equal(hit,expected), path.name
        max_error=float(np.max(np.abs(mesh_ranges[hit]-brep_ranges[hit])))
        assert max_error<1e-6, (path.name,max_error)
        lidar['scan_plane_meshes']=[str(p) for p in mesh_paths]
        _,spec=mount("<sdf><model><link name='chassis'/></model></sdf>",profile,.0325,.06,[-.1005,0.,.14],.05,False)
        filtered=[]
        # 각도 정밀도 비교 표본은 한 스캔 상한보다 많다. 실제 수신 계약을 늘리지 않고 분할 검사한다.
        for start in range(0,len(angles),2048):
            subset=mesh_ranges[start:start+2048]; theta=angles[start:start+2048]
            scan=MeasurementScan(tuple(subset),tuple(theta),tuple(np.linspace(0,.1,len(theta))),time_basis='simulation_acquisition')
            filtered.extend(mask_scan(scan,blind_parameter(0.,spec)).ranges)
        filtered=np.array(filtered)
        assert all(np.isnan(filtered[hit])), path.name
        results.append(dict(profile=path.name,triangles=sum(len(t) for t in meshes),
            mesh_section_segments=len(sections),rays=len(angles),hit_rays=int(sum(hit)),
            blocked_degrees=float(sum(hit)*.02),max_mesh_brep_distance_error_m=max_error,
            self_hits_remaining=int(sum(np.isfinite(filtered[hit])))))
        if path.stem.endswith(('nowire','frontwire_4mm')):
            ax=axes[0 if path.stem.endswith('nowire') else 1]
            for i,p in enumerate(polygons):
                closed=np.vstack((p,p[0]))*1000
                ax.plot(closed[:,0],closed[:,1],color='#ba6b21',lw=2)
            ax.scatter(*(origin*1000),c='#146fb3',label='C1 axis')
            for sector in json.loads(spec)['sectors_deg']:
                for theta in np.deg2rad([sector[0]-sector[1],sector[0]+sector[1]]):
                    line=np.array([origin,origin+.3*np.array([np.cos(theta),np.sin(theta)])])*1000
                    ax.plot(line[:,0],line[:,1],color='#b55',lw=.7,alpha=.6)
            ax.plot([-.1005*1000]*2,[-25,25],color='#7135b0',lw=4,label='ID49 at Z140 (not in pillar audit)')
            ax.set(title='No wire' if path.stem.endswith('nowire') else 'Front wire +4 mm',xlabel='Forward X (mm)',ylabel='Left Y (mm)',
                   xlim=(-150,120),ylim=(-125,125),aspect='equal')
            ax.grid(alpha=.2);ax.legend(fontsize=7)
    report=dict(source_sha256=source['sha256'],source_hash_verified=True,
        scope='static ray/STL comparison, no Gazebo or physical proof',profiles=results)
    (OUT/'geometry_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    figure.savefig(OUT/'mount_sections.png',dpi=150)
    print(json.dumps(report))


if __name__=='__main__':main()
