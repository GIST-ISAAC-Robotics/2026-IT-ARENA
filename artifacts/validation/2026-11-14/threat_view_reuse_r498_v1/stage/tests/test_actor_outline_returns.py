"""오목 외형 반환 분석이 원래 box를 오목 외형으로 잘못 인정하지 않는지 확인."""
import json
import math

import pytest

from audit_actor_outline_returns import evaluate


@pytest.mark.parametrize('range_m,expected', [(.19, True), (.165, False)])
def test_actor_frame_projection_distinguishes_recess_from_box(tmp_path, range_m, expected):
    (tmp_path/'report.json').write_text(json.dumps({'evaluation': {'t_green': 0.}}))
    for name, pose in [('ego', [0., -.3, math.pi/2]), ('actor', [0., 0., 0.])]:
        (tmp_path/f'{name}_poses.jsonl').write_text(''.join(json.dumps([t, *pose])+'\n' for t in (0., 10.)))
    rows = [dict(kind='header', data={'motion_config': {'lidar_x_m': .06, 'lidar_y_m': 0., 'lidar_yaw_rad': 0.}})]
    for nanosec in (0, 0, 100000000):
        scan = dict(sec=4, nanosec=nanosec, ranges=[range_m], angle_min=0., angle_increment=.01,
                    time_increment=.001, range_min=.05, range_max=12.)
        rows.append(dict(kind='call', data={'input': {'scan': scan}}))
    (tmp_path/'follow_decisions.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    result = evaluate(tmp_path)
    assert result['recessed_visual_observed'] is expected
    assert result['unique_scans'] == 2 and result['near_actor_returns'] == 2
    assert result['recessed_center_returns'] == (2 if expected else 0)
    assert result['old_box_center_side_returns'] == (0 if expected else 2)
