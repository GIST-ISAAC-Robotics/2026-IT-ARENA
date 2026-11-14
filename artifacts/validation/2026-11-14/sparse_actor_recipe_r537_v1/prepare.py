"""Windows 정적 생성/확인만 수행한다. ROS/Gazebo/장치 실행은 없다."""
import ast,copy,hashlib,json,math,re
from pathlib import Path
import shutil,sys
import xml.etree.ElementTree as ET
import numpy as np
import cv2
from actor_fixture import A,ROOT,MARKER,ACTOR_OVERRIDES,attach_marker

sys.path.insert(0,str(ROOT/'src/arena_autonomy'))
from arena_autonomy.vehicle_marker_geometry import validate_profile,rectangle,pose_region,camera_matrix

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(name,data):(A/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def functions(path,names,namespace):
    module=ast.parse(path.read_text(encoding='utf-8'))
    selected=[node for node in module.body if isinstance(node,ast.FunctionDef) and node.name in names]
    assert {n.name for n in selected}==set(names)
    exec(compile(ast.Module(body=selected,type_ignores=[]),str(path),'exec'),namespace)
    return namespace

def main():
    base=ROOT/'artifacts/validation/2026-11-14/field_wall_resource_drive_r530_v1'
    prior=ROOT/'artifacts/validation/2026-11-13/sparse_upper_r341_v1'
    source=ROOT/'artifacts/validation/2026-11-13/vehicle_marker_r466_v1/sources/kaist_submitted_image.bin'
    profile_path=ROOT/'config/perception/vehicle_marker_approx_b0390.json'
    profile=validate_profile(json.loads(profile_path.read_text(encoding='utf-8')))
    target=next(row for row in profile['targets'] if (row['dictionary'],row['id'])==(MARKER['dictionary'],MARKER['id']))
    assert target['id'] not in (0,20,30,45,49)
    raw=source.read_bytes();image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_GRAYSCALE)
    ys,xs=np.where(image<128);bbox=[int(xs.min()),int(ys.min()),int(np.ptp(xs)+1),int(np.ptp(ys)+1)]
    assert image.shape==(1000,1000) and bbox==[152,152,696,696]
    assert math.isclose(.05*bbox[2]/image.shape[1],target['black_code_size_m'],abs_tol=1e-12)
    from experimental_facilities import marker_cells
    centers=(152+(np.arange(6)+.5)*696/6).astype(int)
    assert np.array_equal(image[np.ix_(centers,centers)]<128,marker_cells(1)==0)
    detected=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)).detectMarkers(image)[1]
    assert detected.flatten().tolist()==[1]
    shutil.copyfile(source,A/'marker_source.png')  # 원 제출 이미지 byte-identical 보존
    common=ROOT/'scripts/validate_g1_traffic.py'
    factory=functions(common,['actor_sdf'],dict(ET=ET,math=math,CONTACT_TOPIC='/eval/g1b_actor/contact'))['actor_sdf']
    config=json.loads((base/'config.json').read_text(encoding='utf-8'))
    config['registered_at']='2026-11-14 r537 static preparation; execution not started'
    config['scope']='기존 r530 12sim초 단일 상대 일정의 희소 상부 대표 가정. 후면 마커 고정, 보조 OFF/ON 분리. 실물 형상 아님.'
    config['launch']['entry_g1']=(A/'marker_course.launch.py').relative_to(ROOT).as_posix()
    config['actor'].update(ACTOR_OVERRIDES)
    config['actor']['visible_section']='C1 z134.6mm: 8x8mm 지주4개. 하부200x150x80mm/덮개는 광선 높이 밖. 후면50mm 판 중심200mm도 광선 밖.'
    config['actor']['physics']+=' Upper sparse visuals use the existing full conservative collision envelope; not exact component contact geometry.'
    write('config.json',config)
    unmarked=factory(config,(0.,0.,0.))
    assert unmarked==(prior/'sparse_upper_posts.sdf').read_text(encoding='utf-8')
    marked=attach_marker(unmarked)
    (A/'actor.sdf').write_text(marked,encoding='utf-8')
    root=ET.fromstring(marked);link=root.find('model/link');model_z=.13
    collisions=root.findall('model/link/collision')
    assert len(collisions)==1 and ET.tostring(collisions[0])==ET.tostring(ET.fromstring(unmarked).find('model/link/collision'))
    assert not root.findall('.//uri') and not root.findall('.//diffuse_map')
    for visual in link.findall('visual'):
        x,y,z,roll,pitch,yaw=map(float,visual.findtext('pose','0 0 0 0 0 0').split())
        sx,sy,sz=map(float,visual.findtext('geometry/box/size').split())
        assert abs(x)+sx/2<=.1+1e-12 and abs(y)+sy/2<=.075+1e-12
        assert abs(z)+sz/2<=.11+1e-12 and roll==pitch==0.
    ink=link.findall("visual[@name='r537_marker_ink_0_0']")
    assert len(ink)==1
    ink_boxes=[]
    for visual in link.findall('visual'):
        if not visual.get('name').startswith('r537_marker_ink_'):continue
        xyz=np.asarray(list(map(float,visual.findtext('pose').split()[:3])))
        size=np.asarray(list(map(float,visual.findtext('geometry/box/size').split())))
        ink_boxes.append((xyz-size/2,xyz+size/2))
    ink_low=np.min([b[0] for b in ink_boxes],axis=0);ink_high=np.max([b[1] for b in ink_boxes],axis=0)
    np.testing.assert_allclose(ink_low,[-.1,-.0348/2,.07-.0348/2],atol=1e-10)
    np.testing.assert_allclose(ink_high,[-.0996,.0348/2,.07+.0348/2],atol=1e-10)
    audit=functions(prior/'audit.py',['visual_boxes','ranges'],dict(np=np,math=math))
    # 기존 도우미는 yaw=0 box만 받는다. marker는134.6mm보다 높아 이 단면에서 제외한다.
    ray_root=copy.deepcopy(root)
    for visual in list(ray_root.find('model/link').findall('visual')):
        if visual.get('name').startswith('r537_marker_'):ray_root.find('model/link').remove(visual)
    boxes=audit['visual_boxes'](ray_root,.1346)
    assert len(boxes)==4 and math.isclose(sum((b[2]-b[0])*(b[3]-b[1]) for b in boxes),4*.008**2,abs_tol=1e-12)
    for visual in link.findall('visual'):
        if not visual.get('name').startswith('r537_marker_'):continue
        z=float(visual.findtext('pose').split()[2])+model_z
        height=float(visual.findtext('geometry/box/size').split()[2])
        assert z-height/2>.1346
    angles_path=ROOT/'config/tests/c1_standard10_native_angles_v1.json'
    templates=json.loads(angles_path.read_text(encoding='utf-8'))
    q14=templates['patterns_q14'][0];angles=math.pi-np.asarray(q14)*math.pi/32768
    rows=[]
    for cx,cy in ((.5,-.23),(.8,0.),(1.2,0.)):
        dx=cx-.0754;bearing=math.atan2(cy,dx);distance=math.hypot(dx,cy)
        sparse=audit['ranges'](boxes,angles-bearing,distance,-bearing)
        envelope=audit['ranges']([(-.1,-.075,.1,.075)],angles-bearing,distance,-bearing)
        rows.append(dict(actor_center_xy_m=[cx,cy],sensor_xy_m=[.0754,0.],center_distance_m=distance,
            original_rays=len(angles),sparse_returns=int(np.isfinite(sparse).sum()),
            full_envelope_returns=int(np.isfinite(envelope).sum()),
            sparse_source_indices=np.flatnonzero(np.isfinite(sparse)).tolist()))
    mount_path=ROOT/'src/arena_description/config/b0390_c1_cad_20261113_final_frontwire_6mm.yaml'
    mount_text=mount_path.read_text(encoding='utf-8')
    section=mount_text.split('  scan_plane_sections:',1)[1].split('  scan_plane_meshes:',1)[0]
    # 이 파일의 명시 polygon 배열에서 X좌표만 읽는다. YAML 전체를 재해석하지 않는다.
    vertices_x=[float(v) for v in re.findall(r'^\s*(?:-\s+){2,3}(-?\d+(?:\.\d+)?)\s*$',section,re.M)]
    assert len(vertices_x)>20 and max(vertices_x)+.003+.002<.0754
    assert all(np.cos(angles[i])>0 for row in rows for i in row['sparse_source_indices'])
    expected=np.array([[-.1,-.075],[.1,-.075],[.1,.075],[-.1,.075]])
    np.testing.assert_allclose(rectangle([-.1,0.],0.),expected,atol=1e-12)
    # 정답을 넣는 것은 이 오프라인 기하 검사뿐이며 제어/ROS로 전달하지 않는다.
    half=MARKER['black_m']/2;camera=profile['camera'];K=camera_matrix(camera,1280,720)
    body=np.array([[.7,+half,.2+half],[.7,-half,.2+half],[.7,-half,.2-half],[.7,+half,.2-half]])
    delta=body-np.array(camera['base_xyz_m']);optical=np.column_stack((-delta[:,1],-delta[:,2],delta[:,0]))
    pixels=optical[:,:2]/optical[:,2,None]*[K[0,0],K[1,1]]+[K[0,2],K[1,2]]
    region,reason=pose_region(pixels,target,profile,1280,720)
    assert region is not None,(reason,pixels)
    np.testing.assert_allclose(region['rear_center_xy_m'],[.7,0.],atol=1e-7)
    np.testing.assert_allclose(region['polygon_xy_m'],expected+[.8,0.],atol=1e-7)
    report=dict(scope='Windows static geometry/resource audit only; no new render, ROS, Gazebo or device calls',
        actor_shape='existing sparse_upper_posts; unmarked SDF byte-identical to r341',
        actual_opponent_shape='unknown; all dimensions are sensitivity assumptions',
        lidar_z_m=.1346,lower_body_world_z_m=[.02,.10],posts_world_z_m=[.10,.24],top_world_z_m=[.236,.24],
        upper_section_boxes=boxes,upper_section_area_m2=4*.008**2,
        conservative_collision_box_m=[.2,.15,.22],collision_matches_component_volume=False,
        collision_preserved=True,planar_lower_body_matches_evaluation_rectangle=True,
        marker=MARKER,marker_image_sha256=sha(source),marker_image_bbox_px=bbox,
        marker_code_size_m=target['black_code_size_m'],white_margin_each_side_m=(.05-.0348)/2,
        source_image_dictionary_id_confirmed=[MARKER['dictionary'],int(detected.reshape(-1)[0])],
        runtime_render_detection_tested=False,marker_above_lidar=True,
        native_angular_pattern_index=0,native_pattern_is_measured_angles_only=True,
        nominal_mount_max_x_m=max(vertices_x),nominal_mount_with_3mm_motion_2mm_return_margin_max_x_m=max(vertices_x)+.005,
        represented_forward_rays_do_not_intersect_nominal_self_sections=True,
        hardware_acquisition_timing_verified=False,ray_cases=rows,
        marker_rear_point_xy_m=[-.1,0.],marker_inferred_box_error_m=float(np.max(np.abs(np.asarray(region['polygon_xy_m'])-(expected+[.8,0.])))),
        external_sdf_resources=[],actor_sdf_sha256=sha(A/'actor.sdf'),
        source_sha256={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in
            (common,ROOT/'scripts/traffic_actors.py',ROOT/'scripts/audit_actor_outline_returns.py',
             ROOT/'scripts/experimental_facilities.py',profile_path,source,angles_path,mount_path,prior/'sparse_upper_posts.sdf',prior/'audit.py')})
    write('audit.json',report)
    recipe=dict(status='prepared only; root integrates after r530; no overlapping run',single_actor=True,
        base_recipe=str(base.relative_to(ROOT)).replace('\\','/'),case='right_side_start',seed=0,duration_sim_s=12.,
        actor_config='config.json',actor_adapter='actor_fixture.wrap_actor_factory(existing_factory)',
        actor_sdf_reference='actor.sdf',evaluation_schedule_and_planar_envelope_unchanged=True,
        same_marked_actor_for_off_and_on=True,
        off=dict(vehicle_marker_profile_path='',vehicle_marker_observer_enabled=False),
        on=dict(vehicle_marker_profile_path='config/perception/vehicle_marker_approx_b0390.json',vision_execution='process'),
        parameter_delivery='marker_course.launch.py: r537_marker_profile argument -> SetParameter before frozen course include; common CLI unchanged',
        postrun_evidence=['raw current C1 returns at134.6mm','camera image+image stamp','vehicle marker regions from actual image only','drive/safe and guard reason','actor/ego contact and planar gap evaluation only','STOP and process cleanup'],
        no_truth_to_controller=True,render_and_off_on_driving_not_executed=True)
    write('recipe.json',recipe)
    print(json.dumps({k:report[k] for k in ('actor_sdf_sha256','marker_image_bbox_px','marker_code_size_m','ray_cases','marker_inferred_box_error_m')},ensure_ascii=False))

if __name__=='__main__':main()
