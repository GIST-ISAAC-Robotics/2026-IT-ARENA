import run_windows_tests
import sys
sys.path.insert(0,str(run_windows_tests.ROOT/'stage/tests'))
from test_vehicle_marker_regions import projected,profile
from arena_autonomy import vehicle_marker_geometry as g
import cv2,numpy as np,json
p=profile.__wrapped__();q=projected(p,.65,.3,.04);half=.0348/2
obj=np.array([[-half,half,0.],[half,half,0.],[half,-half,0.],[-half,-half,0.]])
k=g.camera_matrix(p['camera'],1280,720)
r=cv2.solvePnPGeneric(obj,q,k,None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
rows=[]
for rv,t in zip(r[1],r[2]):
    rot=cv2.Rodrigues(rv)[0]
    error=np.sqrt(np.mean(np.sum((cv2.projectPoints(obj,rv,t,k,None)[0].reshape(4,2)-q)**2,axis=1)))
    rr,tt=cv2.solvePnPRefineLM(obj,q,k,None,rv.copy(),t.copy())
    rows.append(dict(raw_r=rot.tolist(),raw_t=t.tolist(),error=error,refined_t=tt.tolist(),refined_r=cv2.Rodrigues(rr)[0].tolist()))
print(json.dumps(rows,indent=2))
