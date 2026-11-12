"""CAD 추정 장착 mesh/단면의 시뮬 표시와 범위 한정 자기 반환 계약. 주행 정책 아님."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def bounds(points,origin,angular_margin,distance_margin):
    if len(points)<3 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('finite polygon points required')
    x,y=origin
    angles=[math.atan2(py-y,px-x) for px,py in points]
    ref=math.atan2(sum(py-y for _,py in points),sum(px-x for px,_ in points))
    spread=[math.remainder(a-ref,math.tau) for a in angles]
    lo,hi=min(spread),max(spread)
    center=math.remainder(ref+(lo+hi)/2,math.tau)
    half=math.degrees((hi-lo)/2)+angular_margin
    if not 0<half<90:raise ValueError('polygon overlaps sensor or exceeds local sector')
    nearest=math.inf
    for a,b in zip(points,points[1:]+points[:1]):
        dx,dy=b[0]-a[0],b[1]-a[1];d2=dx*dx+dy*dy
        t=max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/d2)) if d2 else 0.
        nearest=min(nearest,math.hypot(a[0]+t*dx-x,a[1]+t*dy-y))
    farthest=max(math.hypot(px-x,py-y) for px,py in points)
    if nearest<=distance_margin:raise ValueError('structure touches sensor')
    return [math.degrees(center),half],[nearest-distance_margin,farthest+distance_margin]


def mount(model_xml,profile,wheel_radius,body_height,marker_xyz,marker_size,marker_enabled):
    lidar=profile['lidar'];sx,sy,sz=map(float,lidar['xyz_m'])
    margin=float(lidar.get('known_structure_margin_deg',.5))
    distance_margin=float(lidar['self_return_distance_margin_m'])
    if not 0<=margin<=5 or not 0<=distance_margin<=.01:raise ValueError('invalid CAD mask margins')
    if lidar.get('scan_plane_posts') or lidar.get('scan_plane_wires'):
        raise ValueError('CAD section and legacy cylinders cannot be combined')
    if lidar.get('known_structure_mask') not in ('unknown','structure_aware'):
        raise ValueError('CAD model requires explicit unknown/structure policy')
    root=ET.fromstring(model_xml);chassis=root.find("model/link[@name='chassis']")
    chassis_z=wheel_radius+body_height/2
    for index,uri in enumerate(lidar['scan_plane_meshes']):
        path=Path(uri)
        if not path.is_absolute() or not path.is_file():raise ValueError('resolved CAD mesh required')
        visual=ET.SubElement(chassis,'visual',name=f'c1_cad_mount_{index}')
        ET.SubElement(visual,'pose').text=f'0 0 {-chassis_z} 0 0 0'
        mesh=ET.SubElement(ET.SubElement(visual,'geometry'),'mesh')
        ET.SubElement(mesh,'uri').text=path.as_uri();ET.SubElement(mesh,'scale').text='1 1 1'
        material=ET.SubElement(visual,'material')
        for key in ('ambient','diffuse'):ET.SubElement(material,key).text='0.7 0.42 0.18 1'
    sectors=[];bands=[]
    for row in lidar['scan_plane_sections']:
        # 각 기둥과 전방/안쪽 부착면의 실제 꼭짓점 각도 범위. 전 방향 외접원 없음.
        points=list(row['polygon_xy_m'])
        for polygon in row.get('wire_polygons_xy_m',[]):points.extend(polygon)
        # 거리 하한은 각 실제 polygon edge에서 얻는다. 별개 polygon을 잇는 가상 edge는 쓰지 않는다.
        sector,_=bounds(points,(sx,sy),margin,distance_margin)
        separate=[bounds(p,(sx,sy),margin,distance_margin)[1]
                  for p in [row['polygon_xy_m'],*row.get('wire_polygons_xy_m',[])]]
        sectors.append(sector);bands.append([min(b[0] for b in separate),max(b[1] for b in separate)])
    mx,my,mz=marker_xyz
    if marker_enabled and abs(mz-sz)<=marker_size/2:
        # 기존 후면 판 두께1mm와 폭50mm. 높이는 호출자의 실제 시험 선택을 그대로 따른다.
        points=[[mx-.0005,my-marker_size/2],[mx+.0005,my-marker_size/2],
                [mx+.0005,my+marker_size/2],[mx-.0005,my+marker_size/2]]
        sector,band=bounds(points,(sx,sy),margin,distance_margin);sectors.append(sector);bands.append(band)
    return ET.tostring(root,encoding='unicode'),json.dumps({'sectors_deg':sectors,'range_limits_m':bands})
