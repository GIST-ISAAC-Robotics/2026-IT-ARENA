"""D2 저장 반환에서 좌우 경로 형상을 비교한다. 정답은 결과 위치 표시에만 사용한다."""
import json
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call, motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_path import local_path

A = Path(__file__).resolve().parent
source = A/'d_markers_blank2/follow_decisions.jsonl'
rows, last = [], None
for line in source.open():
    event = json.loads(line)
    if event['kind'] == 'header':
        mc = MotionConfig(**event['data']['motion_config'])
        cfg = event['data']['follow_config']
    if event['kind'] != 'call':
        continue
    raw, out = event['data']['input'], event['data']['output']
    t = raw['reference_s']
    scan_id = (raw['scan']['sec'], raw['scan']['nanosec'])
    if not 28.8 <= t <= 31.1 or scan_id == last:
        continue
    last = scan_id
    data, scan, history = restore_call(raw, mc)
    scene = motion_scene(scan, history, t, timing_verified=data['timing_verified'],
                         epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
    cloud = np.asarray(scene.occupied_endpoints_xy_m)
    paths = {}
    for side in ('left', 'right'):
        path, info = local_path(cloud, side, cfg['wall_offset_m'], horizon=3.,
             piecewise_corner=True, minimum_radius=cfg['wheelbase_m']/np.tan(cfg['max_steering_rad']),
             narrow_occluder_bridge=True)
        values = None
        if path is not None:
            # 最近傍 표본의 (x,y). 외삽하지 않는다.
            values = [[round(float(v),3) for v in path[np.argmin(abs(path[:,0]-x))]]
                      for x in (.1,.3,.5,.7,1.,1.3)]
        paths[side] = dict(reason=info['reason'], model=info.get('wall_model'),
            ahead=info.get('boundary_observed_ahead_m'), anchor=info.get('path_anchor_error_m'), points=values)
    det = out[2]
    rows.append(dict(t=t, speed=out[0], steering=out[1], reason=det.get('reason'),
        selected=det.get('boundary_selection'), wall=det.get('path_wall'),
        accepted=det.get('fresh_boundary_accepted'), reference=det.get('bilateral_reference'), paths=paths))
(A/'d2_path_probe.json').write_text(json.dumps(rows, indent=2)+'\n')
for r in rows:
    print(round(r['t'],2), round(r['speed'],2), r['selected'], r['wall'], r['reference'],
          {s:(p['model'],None if p['ahead'] is None else round(p['ahead'],2),p['points']) for s,p in r['paths'].items()})
