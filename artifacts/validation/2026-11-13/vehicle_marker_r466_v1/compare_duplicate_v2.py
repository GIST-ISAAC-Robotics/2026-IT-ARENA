"""이미 확보한 합성 영상/quad의 선택·거리 차이를 보존한다. 실제거리 계측이 아니다."""
import run_windows_tests as bootstrap
import json,sys
sys.path.insert(0,str(bootstrap.ROOT/'stage/tests'))
from test_vehicle_marker_duplicates import image,OUTER,INNER
from arena_autonomy import vehicle_marker_geometry as g
import cv2,numpy as np
root=bootstrap.ROOT;profile=json.loads((root/'stage/config/perception/vehicle_marker_approx_b0390.json').read_bytes())
target=profile['targets'][0];bigger=OUTER.mean(axis=0)+(OUTER-OUTER.mean(axis=0))*(89/79)
frame_quad=OUTER.mean(axis=0)+(OUTER-OUTER.mean(axis=0))*(91/79)
cases=[]
for name,rgb,quads in [('original_4_6_nested',image(),[OUTER,INNER]),
        ('larger_white_board',image(),[bigger,OUTER]),('larger_black_frame',image(frame=True),[frame_quad,OUTER])]:
    gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);items=[]
    selected,resolution=g.representative_quad(rgb,quads,target)
    for q in quads:
        pose,reason=g.pose_region(q,target,profile,1280,720)
        items.append(dict(corners_px=q.tolist(),black_code_evidence=g._black_code_evidence(gray,q,target),
            camera_depth_m=None if pose is None else pose['rear_center_xy_m'][0]-.055,
            reprojection_error_px=None if pose is None else pose['reprojection_error_px'],pose_error=reason))
    cases.append(dict(name=name,candidates=items,selected_quad=selected.tolist(),resolution=resolution))
    (root/'revision_v2'/(name+'.png')).write_bytes(cv2.imencode('.png',cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))[1].tobytes())
(root/'revision_v2/quad_comparison.json').write_text(json.dumps(dict(scope='synthetic only; absolute camera/print accuracy unverified',cases=cases),indent=2)+'\n')
print(json.dumps(cases,indent=2))
