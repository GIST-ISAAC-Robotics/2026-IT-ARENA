"""새 지지 틀만 선택형 시험 SDF로 분리. 공식 광원/마커/월드는 수정하지 않는다."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
from shapely.geometry import LineString
from shapely.ops import polygonize, unary_union
from shapely.affinity import rotate, translate

ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'scripts'))
from surface_contact_eval import SurfaceMap
from build_cad_sensor_profile import write_stl

spec=importlib.util.spec_from_file_location('revision_analysis',OUT/'analyze_revision.py')
analysis=importlib.util.module_from_spec(spec);spec.loader.exec_module(analysis)


def section(tris,height):
    lines=analysis.audit.slice_xy(tris,2,height)
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


def main():
    raw=OUT/'STardium_starting_light_new.stl'
    assert hashlib.sha256(raw.read_bytes()).hexdigest()=='04f43b4499cd32b2bbb27b74eb14735df6ff9c069676157a58f33543d7b1856f'
    _,_,tri,_=analysis.audit.load(raw)
    lo=tri.reshape(-1,3).min(0);hi=tri.reshape(-1,3).max(0)
    # 이전/새 STL 모두 같은 원 좌표/외곽의 광원 모듈7033면이다. 표시/충돌 중복을 피한다.
    module=np.all((tri[:,:,0]>-110)&(tri[:,:,0]<90)&(tri[:,:,1]>-60)&(tri[:,:,1]<16),axis=1)
    assert int(sum(module))==7033
    assert np.min(tri[module,:,1]-lo[1])>.38*1000
    supports=tri[~module]
    center=(lo+hi)/2
    # +X진행, +Y좌측, +Z위. cyclic permutation은 원 삼각형 순서(det+1)를 보존한다.
    transformed=np.stack((supports[:,:,2]-center[2],supports[:,:,0]-center[0],supports[:,:,1]-lo[1]),axis=2)/1000
    mesh=OUT/'support_frame_m.stl';write_stl(mesh,transformed)
    levels=(.0025,.05,.075,.1346,.198363,.35)
    sections={z:section(transformed,z) for z in levels}
    world=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf'
    surface=SurfaceMap.from_world(world)
    locations=[dict(name='signal',x=9.8,y=6.145,yaw=math.pi/2,basis='existing signal centre'),
               dict(name='marker20',x=6.5275,y=6.,yaw=0.,basis='previous provisional centre +372.5mm upstream of ID20'),
               dict(name='marker30',x=2.48,y=3.505,yaw=math.pi/2,basis='existing ID30 centre')]
    root=ET.Element('sdf',version='1.10');rows=[]
    for place in locations:
        model=ET.SubElement(root,'model',name='eval_20261112_'+place['name']+'_support')
        ET.SubElement(model,'static').text='true'
        ET.SubElement(model,'pose').text=f"{place['x']} {place['y']} 0 0 0 {place['yaw']}"
        link=ET.SubElement(model,'link',name='frame')
        for kind in ('collision','visual'):
            element=ET.SubElement(link,kind,name='support_'+kind)
            geom=ET.SubElement(ET.SubElement(element,'geometry'),'mesh')
            ET.SubElement(geom,'uri').text=str(mesh.resolve());ET.SubElement(geom,'scale').text='1 1 1'
            if kind=='visual':
                material=ET.SubElement(element,'material')
                for tag in ('ambient','diffuse'):ET.SubElement(material,tag).text='.25 .25 .25 1'
        footprint=[]
        for height,shape in sections.items():
            placed=translate(rotate(shape,place['yaw'],origin=(0,0),use_radians=True),place['x'],place['y'])
            footprint.append(dict(height_m=height,material_area_m2=placed.area,
                road_overlap_m2=placed.intersection(surface.road).area,
                wall_overlap_m2=placed.intersection(surface.walls).area,
                polygons=len(placed.geoms) if hasattr(placed,'geoms') else 1))
        rows.append(dict(**place,sections=footprint))
    ET.indent(root);ET.ElementTree(root).write(OUT/'supports_trial.sdf',encoding='utf8',xml_declaration=True)
    report=dict(scope='candidate static mesh fixture; not installed in runtime; markers use user assumption',
        source_sha256=hashlib.sha256(raw.read_bytes()).hexdigest(),source_world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        removed_module_triangles=int(sum(module)),support_triangles=len(supports),
        mesh_sha256=hashlib.sha256(mesh.read_bytes()).hexdigest(),
        transform_raw_mm_to_metres=dict(forward='raw Z minus depth centre',left='raw X minus width centre',up='raw Y minus ground minimum',
            raw_width_centre_mm=center[0],raw_depth_centre_mm=center[2],raw_ground_y_mm=lo[1]),
        lighting='existing dynamic panel and 2x5 retained; STL raw module unchanged but ground reference shifts 9mm; height not auto-adopted',
        locations=rows)
    (OUT/'fixture.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from shapely.geometry import box
    fig,axes=plt.subplots(1,3,figsize=(15,6),layout='constrained')
    def polygons(shape):
        if shape.is_empty:return []
        if shape.geom_type=='Polygon':return [shape]
        return [p for g in getattr(shape,'geoms',[]) for p in polygons(g)]
    for ax,place in zip(axes,locations):
        x,y=place['x'],place['y'];window=box(x-.8,y-.8,x+.8,y+.8)
        for shape,color in [(surface.road,'#eeeeee'),(surface.walls,'#333333')]:
            for poly in polygons(shape.intersection(window)):
                ax.fill(*poly.exterior.xy,color=color)
        for height,color,label in [(.0025,'#c17b13','Foot 2.5 mm'),(.1346,'#b82035','Scan 134.6 mm')]:
            shape=translate(rotate(sections[height],place['yaw'],origin=(0,0),use_radians=True),x,y)
            for i,poly in enumerate(polygons(shape)):
                ax.plot(*poly.exterior.xy,color=color,lw=2,label=label if i==0 else None)
        ax.scatter([x],[y],s=25,c='#2053a8',label='Frame centre')
        ax.set(title=place['name'],xlim=(x-.8,x+.8),ylim=(y-.8,y+.8),aspect='equal',xlabel='World X (m)',ylabel='World Y (m)')
        ax.set_facecolor('#cfe7c7');ax.legend(fontsize=7);ax.grid(alpha=.15)
    fig.savefig(OUT/'placement_sections.png',dpi=150)
    print(json.dumps(report))


if __name__=='__main__':main()
