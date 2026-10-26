import json, sys, math
import numpy as np
D='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-24/m3_p1_offline_v1'
sys.path[:0]=[D]
import screen as s, candidate as c
motion, follow, calls = s.load_inputs('artifacts/validation/2026-10-24/e3_onroad_v1/g1')
for e in calls:
    if e['kind']=='call' and e['data']['input']['reference_s'] and abs(e['data']['input']['reference_s']-3.96)<1e-6:
        data, scene, pts, m, b, st, rb, ages = s.scene_inputs(e, motion, follow)
        path=np.asarray(e['data']['output'][2]['path_points'])
        print('path first/last', path[0], path[-1], 'prev', data['previous_steering_rad'])
        state=c.side_state(pts,b,st,path)
        print('ego_l', state['ego_offset_m'], 'psi', state['yaw_from_path_rad'])
        p1=c.p1_path(path,state['ego_offset_m'],state['half_span_m'])
        print('p1 first', p1['path'][:3], 'near x0', p1['path'][np.argmin(np.abs(p1['path'][:,0]))])
        from arena_autonomy.local_follow import pursue_path
        print('desired p1', pursue_path(p1['path'], data['speed_mps'], follow, .1))
        print('desired p0', pursue_path(path, data['speed_mps'], follow, .1))
