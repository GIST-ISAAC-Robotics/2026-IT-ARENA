"""공식 등록 이미지 비율 + 현재 CAD 명목값. 실측 교정값을 만들지 않는다."""
import copy
import json
from pathlib import Path

root=Path(__file__).resolve().parent
dest=root/'stage/config/perception'
dest.mkdir(parents=True,exist_ok=True)
def target(team,dictionary,identity,size,basis,comment):
    return dict(team=team,dictionary=dictionary,id=identity,black_code_size_m=size,
        printed_image_size_m=.05,size_basis=basis,
        source_url=f'https://github.com/MOSW626/istech-it-arena/issues/1#issuecomment-{comment}')
profile=dict(schema=1,profile_id='b0390_cad_vehicle_marker_approx_v1',mode='approximate_v1',
    purpose='occupancy_assist',camera=dict(width_px=1280,height_px=720,horizontal_fov_deg=62.2,
        optical_frame_id='camera_color_optical_frame',base_xyz_m=[.055,0.,.198363],
        base_rpy_rad=[0.,0.,0.],distortion=[],source='B0390 nominal HFOV + CAD; installed calibration unverified'),
    own_marker=dict(dictionary='DICT_4X4_50',id=49),range_scale=1.,heading_offset_rad=0.,
    max_age_s=.25,max_range_m=5.,max_reprojection_error_px=4.,
    virtual_extent_m=dict(width=.15,length=.20),accuracy_verified=False,
    official_commit='6c2589c61acb83928339291ae237180b17c2cb54',targets=[
        target('KAIST','DICT_4X4_50',1,.0348,'submitted PNG black 696 / whole 1000 px; assumed 50 mm whole print',5828019968),
        target('UNIST','DICT_4X4_50',7,.0348,'submitted PNG black 696 / whole 1000 px; assumed 50 mm whole print',5772491077),
        target('DGIST','DICT_4X4_50',11,.05*590/710,'edited submitted PNG black 590 / whole 710 px; organizer table still says 14',5978592693),
        target('POSTECH','DICT_7X7_100',91,.05,'submitted SVG width 50 mm black viewBox 9/9; no outer white quiet zone',5779593670)],
    assumptions=['마커가 후방 중앙에 있고 코드의 위가 차량의 위인 부착을 가정합니다.',
        '실제 출력물의 검은 코드 외곽/흰 여백은 아직 측정하지 않았습니다.',
        'DGIST 등록 이미지 11과 주최 표 14가 충돌하며 14를 임의 추가하지 않습니다.',
        'POSTECH의 흰 여백을 밖에 추가하면 코드 50 mm 유지, 전체를 50 mm로 축소하면 코드 크기 재입력이 필요합니다.',
        '빈 왜곡 계수는 왜곡 없음의 검증이 아닌 미교정 근사입니다.'])
(dest/'vehicle_marker_approx_b0390.json').write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
cal=copy.deepcopy(profile)
cal.update(profile_id='b0390_gist49_calibration_only_v1',purpose='calibration_only',targets=[
    target('GIST','DICT_4X4_50',49,.0348,'submitted PNG black 696 / whole 1000 px; assumed 50 mm whole print',5858748518)])
(dest/'vehicle_marker_gist49_calibration_only.json').write_text(json.dumps(cal,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
