"""앞차 속도를 모르는 이동 조각에 물체 이동 가정을 하한으로 쓰는 선택형 후보를 적용한다.

v1 기록을 다시 재생할 수 있도록 값을 덮어쓰지 않고 별도 선택지로 추가한다. 기대 원문이
한 번만 나타나지 않으면 아무 파일도 쓰지 않는다.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
EDITS = {
    'src/arena_autonomy/arena_autonomy/bypass_handoff.py': [
        ("    low_motion_reserve: bool = False\n",
         "    low_motion_reserve: bool = False\n"
         "    # 전진율을 못 얻은 조각은 G1 전체가 아니라 물체 이동 가정만큼만 풀어 우회 공간을 남긴다.\n"
         "    moving_pace_floor: bool = False\n"),
        ("            raise ValueError('low motion reserve requires closing follow evidence without recent forward')\n",
         "            raise ValueError('low motion reserve requires closing follow evidence without recent forward')\n"
         "        if type(self.moving_pace_floor) is not bool or (self.moving_pace_floor and not self.low_motion_reserve):\n"
         "            raise ValueError('moving pace floor requires low motion reserve')\n"),
        ("            return target, dict(result, reason='reserve_waits_low_motion', fixed_reserve_cap_mps=cap,\n"
         "                reserve_m=reserve, rear_axle_to_patch_m=distance)\n",
         "            waiting = dict(result, reason='reserve_waits_low_motion', fixed_reserve_cap_mps=cap,\n"
         "                reserve_m=reserve, rear_axle_to_patch_m=distance)\n"
         "            if not self.execution.moving_pace_floor:\n"
         "                return target, waiting\n"
         "            # 앞차 속도를 모를 때 0으로 두지 않고 G1의 물체 이동 가정을 쓴다. 정확한 속도 측정이 아니며\n"
         "            # 앞차가 그보다 빠르면 간격이 벌어질 뿐이다. 이 거리는 앞차 급정지 뒤의 우회 공간이다.\n"
         "            paced = cap+self.monitor.motion_bound\n"
         "            return min(target,paced), dict(waiting, applied=paced<target, before_mps=target, cap_mps=paced)\n"),
    ],
    'src/arena_autonomy/arena_autonomy/g1_g2_node.py': [
        ("        low_motion_reserve = self.declare_parameter('g1_g2_low_motion_reserve',False).value\n",
         "        low_motion_reserve = self.declare_parameter('g1_g2_low_motion_reserve',False).value\n"
         "        moving_pace_floor = self.declare_parameter('g1_g2_moving_pace_floor',False).value\n"),
        ("            low_motion_reserve=low_motion_reserve,reacquire_policy=reacquire,\n",
         "            low_motion_reserve=low_motion_reserve,moving_pace_floor=moving_pace_floor,reacquire_policy=reacquire,\n"),
    ],
    'src/arena_bringup/launch/simulation.launch.py': [
        ("    if (reserve not in ('fixed','moving','closing','recent','low_motion') or mode != 'local_pursuit' or not guarded\n",
         "    if (reserve not in ('fixed','moving','closing','recent','low_motion','paced') or mode != 'local_pursuit' or not guarded\n"),
        ("                g1_g2_moving_follow_reserve=reserve in ('moving','closing','recent','low_motion'),\n"
         "                g1_g2_closing_follow_reserve=reserve in ('closing','recent','low_motion'),\n"
         "                g1_g2_recent_forward_reserve=reserve=='recent', g1_g2_low_motion_reserve=reserve=='low_motion',\n",
         "                g1_g2_moving_follow_reserve=reserve in ('moving','closing','recent','low_motion','paced'),\n"
         "                g1_g2_closing_follow_reserve=reserve in ('closing','recent','low_motion','paced'),\n"
         "                g1_g2_recent_forward_reserve=reserve=='recent', g1_g2_low_motion_reserve=reserve in ('low_motion','paced'),\n"
         "                g1_g2_moving_pace_floor=reserve=='paced',\n"),
        ('choices=["off","fixed","moving","closing","recent","low_motion"],',
         'choices=["off","fixed","moving","closing","recent","low_motion","paced"],'),
    ],
    'scripts/g1_handoff_eval.py': [
        ("    if (reserve not in ('fixed', 'moving','closing','recent','low_motion') or not recorded or\n",
         "    if (reserve not in ('fixed', 'moving','closing','recent','low_motion','paced') or not recorded or\n"),
        ("        reserve=e.get('moving_follow_reserve') is (reserve in ('moving','closing','recent','low_motion')),\n"
         "        closing=e.get('closing_follow_reserve',False) is (reserve in ('closing','recent','low_motion')),\n"
         "        recent=e.get('recent_forward_reserve',False) is (reserve=='recent'),\n"
         "        low_motion=e.get('low_motion_reserve',False) is (reserve=='low_motion'),\n",
         "        reserve=e.get('moving_follow_reserve') is (reserve in ('moving','closing','recent','low_motion','paced')),\n"
         "        closing=e.get('closing_follow_reserve',False) is (reserve in ('closing','recent','low_motion','paced')),\n"
         "        recent=e.get('recent_forward_reserve',False) is (reserve=='recent'),\n"
         "        low_motion=e.get('low_motion_reserve',False) is (reserve in ('low_motion','paced')),\n"
         "        pace_floor=e.get('moving_pace_floor',False) is (reserve=='paced'),\n"),
    ],
    'scripts/validate_g1_traffic.py': [
        ("    parser.add_argument('--handoff-reserve', choices=('fixed','moving','closing','recent','low_motion'),\n",
         "    parser.add_argument('--handoff-reserve', choices=('fixed','moving','closing','recent','low_motion','paced'),\n"),
    ],
}

pending = {}
for path, pairs in EDITS.items():
    data = (ROOT / path).read_bytes()
    newline = '\r\n' if data.count(b'\r\n') > data.count(b'\n') / 2 else '\n'
    for old, new in pairs:
        old_bytes = old.replace('\n', newline).encode('utf-8')
        new_bytes = new.replace('\n', newline).encode('utf-8')
        if data.count(old_bytes) != 1:
            raise ValueError(f'{path}: 기대 원문이 한 번이 아님: {old[:60]!r}')
        data = data.replace(old_bytes, new_bytes)
    pending[path] = data
for path, data in pending.items():
    (ROOT / path).write_bytes(data)
print('applied', len(pending))
