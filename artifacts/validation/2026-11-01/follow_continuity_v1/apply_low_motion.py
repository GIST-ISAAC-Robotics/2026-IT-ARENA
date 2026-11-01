"""낮은 이동 근거 전 고정 여유를 걸지 않는 선택형 후보를 정확한 바이트 치환으로 적용한다.

이미 적용했거나 기대 원문이 한 번만 나타나지 않으면 아무 파일도 쓰지 않고 멈춘다.
"""
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
FILES = ('src/arena_autonomy/arena_autonomy/bypass_handoff.py',
         'src/arena_autonomy/arena_autonomy/g1_g2_node.py',
         'src/arena_bringup/launch/simulation.launch.py',
         'scripts/g1_handoff_eval.py',
         'scripts/validate_g1_traffic.py')


def encoded(text, newline):
    return text.replace('\n', newline).encode('utf-8')


EDITS = {
    FILES[0]: [
        ("    recent_forward_reserve: bool = False\n",
         "    recent_forward_reserve: bool = False\n"
         "    # 연속 세 스캔의 낮은 이동이 쌓이기 전이나 이동 근거가 남은 동안 고정 우회 여유를 걸지 않는다.\n"
         "    low_motion_reserve: bool = False\n"),
        ("            raise ValueError('recent forward reserve requires closing follow evidence')\n",
         "            raise ValueError('recent forward reserve requires closing follow evidence')\n"
         "        if type(self.low_motion_reserve) is not bool or (self.low_motion_reserve and (\n"
         "                not self.closing_follow_reserve or self.recent_forward_reserve)):\n"
         "            raise ValueError('low motion reserve requires closing follow evidence without recent forward')\n"),
        ("            return dict(result, reason='single_scan_patch')\n",
         "            return dict(result, reason='single_scan_patch')\n"
         "        if self.execution.low_motion_reserve and len(self.monitor.rows) < 3:\n"
         "            # 두 스캔의 6 cm는 0.6 m/s 이하일 뿐이다. 세 스캔(0.3 m/s 이하)이 쌓인 뒤 준비한다.\n"
         "            return dict(result, reason='short_low_motion_history')\n"),
        ("                reserve_m=reserve, rear_axle_to_patch_m=distance)\n"
         "        return min(target,cap), dict(result, applied=cap<target, reason='observed_entry_distance_reserve',\n",
         "                reserve_m=reserve, rear_axle_to_patch_m=distance)\n"
         "        if self.execution.low_motion_reserve and (self.monitor.motion_evidence() or len(self.monitor.rows) < 3):\n"
         "            # 고정 여유는 멈췄을 수 있는 조각의 미래 우회용이다. 전진율을 못 얻은 빠른 조각은\n"
         "            # 전진율 0으로 취급하지 않고, 간격·감속·직접 충돌 판단을 G1 추종에 그대로 맡긴다.\n"
         "            return target, dict(result, reason='reserve_waits_low_motion', fixed_reserve_cap_mps=cap,\n"
         "                reserve_m=reserve, rear_axle_to_patch_m=distance)\n"
         "        return min(target,cap), dict(result, applied=cap<target, reason='observed_entry_distance_reserve',\n"),
        ("            if space and prepare and not persistence['ready']:\n",
         "            # 낮은 이동 후보에서는 위 고정 여유가 1.20 m 앞에서 멈춘다. 별도 0 대기를 겹치지 않는다.\n"
         "            if space and prepare and not persistence['ready'] and not self.execution.low_motion_reserve:\n"),
    ],
    FILES[1]: [
        ("        recent_reserve = self.declare_parameter('g1_g2_recent_forward_reserve',False).value\n",
         "        recent_reserve = self.declare_parameter('g1_g2_recent_forward_reserve',False).value\n"
         "        low_motion_reserve = self.declare_parameter('g1_g2_low_motion_reserve',False).value\n"),
        ("            closing_follow_reserve=closing_reserve,recent_forward_reserve=recent_reserve,reacquire_policy=reacquire,\n",
         "            closing_follow_reserve=closing_reserve,recent_forward_reserve=recent_reserve,\n"
         "            low_motion_reserve=low_motion_reserve,reacquire_policy=reacquire,\n"),
    ],
    FILES[2]: [
        ("    if (reserve not in ('fixed','moving','closing','recent') or mode != 'local_pursuit' or not guarded\n",
         "    if (reserve not in ('fixed','moving','closing','recent','low_motion') or mode != 'local_pursuit' or not guarded\n"),
        ("                g1_g2_moving_follow_reserve=reserve in ('moving','closing','recent'),\n"
         "                g1_g2_closing_follow_reserve=reserve in ('closing','recent'),\n"
         "                g1_g2_recent_forward_reserve=reserve=='recent', g1_g2_reacquire_policy='known_hold_v1',\n",
         "                g1_g2_moving_follow_reserve=reserve in ('moving','closing','recent','low_motion'),\n"
         "                g1_g2_closing_follow_reserve=reserve in ('closing','recent','low_motion'),\n"
         "                g1_g2_recent_forward_reserve=reserve=='recent', g1_g2_low_motion_reserve=reserve=='low_motion',\n"
         "                g1_g2_reacquire_policy='known_hold_v1',\n"),
        ('choices=["off","fixed","moving","closing","recent"],', 'choices=["off","fixed","moving","closing","recent","low_motion"],'),
    ],
    FILES[3]: [
        ("    if (reserve not in ('fixed', 'moving','closing','recent') or not recorded or\n",
         "    if (reserve not in ('fixed', 'moving','closing','recent','low_motion') or not recorded or\n"),
        ("        reserve=e.get('moving_follow_reserve') is (reserve in ('moving','closing','recent')),\n"
         "        closing=e.get('closing_follow_reserve',False) is (reserve in ('closing','recent')),\n"
         "        recent=e.get('recent_forward_reserve',False) is (reserve=='recent'),\n",
         "        reserve=e.get('moving_follow_reserve') is (reserve in ('moving','closing','recent','low_motion')),\n"
         "        closing=e.get('closing_follow_reserve',False) is (reserve in ('closing','recent','low_motion')),\n"
         "        recent=e.get('recent_forward_reserve',False) is (reserve=='recent'),\n"
         "        low_motion=e.get('low_motion_reserve',False) is (reserve=='low_motion'),\n"),
    ],
    FILES[4]: [
        ("    parser.add_argument('--handoff-reserve', choices=('fixed','moving','closing','recent'),\n",
         "    parser.add_argument('--handoff-reserve', choices=('fixed','moving','closing','recent','low_motion'),\n"),
    ],
}

before = BASE / 'before.zip'
if before.exists():
    raise FileExistsError('수정 전 사본은 한 번만 만든다')
pending = {}
for path, pairs in EDITS.items():
    data = (ROOT / path).read_bytes()
    # 파일별 줄바꿈을 그대로 유지한다.
    newline = '\r\n' if data.count(b'\r\n') > data.count(b'\n') / 2 else '\n'
    for old, new in pairs:
        old_bytes = encoded(old, newline)
        new_bytes = encoded(new, newline)
        if data.count(old_bytes) != 1:
            raise ValueError(f'{path}: 기대 원문이 한 번이 아님: {old[:60]!r}')
        data = data.replace(old_bytes, new_bytes)
    pending[path] = data
with zipfile.ZipFile(before, 'x', zipfile.ZIP_DEFLATED) as archive:
    for path in FILES:
        archive.writestr(path, (ROOT / path).read_bytes())
for path, data in pending.items():
    (ROOT / path).write_bytes(data)
print('applied', len(pending))
