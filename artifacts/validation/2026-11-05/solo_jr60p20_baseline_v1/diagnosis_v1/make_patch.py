"""후보 A 최소 변경 초안을 격리 사본 위에 만들어 candidate_A.patch로 남긴다(작업트리 미적용).

기준: ~/arena_scratch/solo_diag_v1_src(run1 등록 해시 일치). 결과 사본: ~/arena_scratch/solo_diag_v1_candA.
각 치환은 정확히 한 번 일치해야 한다. 기본값 OFF이며 boundary_bridge가 켜진 단독 구성에서만 허용된다.
"""
import difflib
import os
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
BASE = Path(os.path.expanduser('~/arena_scratch/solo_diag_v1_src'))
CAND = Path(os.path.expanduser('~/arena_scratch/solo_diag_v1_candA'))
FOLLOW = 'arena_autonomy/arena_autonomy/local_follow.py'
PURSUIT = 'arena_autonomy/arena_autonomy/local_pursuit.py'

EDITS = {FOLLOW: [
    ("    boundary_bridge: bool = False  # 단독 선택형. 점유가 아닌 경로 근거 전환 때문에 생긴 정지만 잇는다.\n",
     "    boundary_bridge: bool = False  # 단독 선택형. 점유가 아닌 경로 근거 전환 때문에 생긴 정지만 잇는다.\n"
     "    bridge_release_continuity: bool = False  # 단독 후보. 회랑 다리 직후 기본 경로의 재개 확인만 남은 호출을 0으로 끊지 않는다.\n"),
    ("            raise ValueError('boundary bridge requires solo route and corridor fallback')\n",
     "            raise ValueError('boundary bridge requires solo route and corridor fallback')\n"
     "        if (type(self.bridge_release_continuity) is not bool or\n"
     "                (self.bridge_release_continuity and not self.boundary_bridge)):\n"
     "            raise ValueError('bridge release continuity requires boundary bridge')\n"),
    ("'boundary_bridge', 'low_speed_arc_reacquire',", "'boundary_bridge', 'bridge_release_continuity', 'low_speed_arc_reacquire',"),
    ("        self.held = True\n        self.hold_reason = None\n",
     "        self.held = True\n        self.hold_reason = None\n        self.bridged_last = False\n"),
    ("                (baseline[2]['reason'] == 'follow_boundary_unconfirmed' or waiting)):\n"
     "            return baseline[0], baseline[1], detail\n",
     "                (baseline[2]['reason'] == 'follow_boundary_unconfirmed' or waiting)):\n"
     "            bridged, self.bridged_last = self.bridged_last, False\n"
     "            if (self.config.bridge_release_continuity and waiting and bridged and self.release_count >= 1\n"
     "                    and baseline[2].get('swept_clearance_m', 0.) >= self.config.resume_clearance_m\n"
     "                    and baseline[2].get('target_speed_before_hold_mps', 0.) > 0.):\n"
     "                # 회랑 다리로 주행하던 중 다리 후보가 기본 경로의 재개 확인보다 먼저 꺼진 호출. 이미 수락한\n"
     "                # 기본 경로의 목표(곡률·제동·점유 반영)를 낸다. 관측·점유 정지 뒤 대기는 잇지 않으며\n"
     "                # 부분 관측 상한·독립 보호층은 그대로 뒤에서 적용된다.\n"
     "                self.bridged_last = True\n"
     "                return (float(baseline[2]['target_speed_before_hold_mps']), baseline[1],\n"
     "                        {**detail, 'reason': 'local_follow_bridge_release', 'behavior': 'FOLLOW',\n"
     "                         'underlying_reason': 'follow_release_wait'})\n"
     "            return baseline[0], baseline[1], detail\n"),
    ("            detail['boundary_selection'] = 'current_short_observed_prefix_candidate'\n"
     "        return proposal['proposed_speed_mps'], proposal['steering_rad'], detail\n",
     "            detail['boundary_selection'] = 'current_short_observed_prefix_candidate'\n"
     "        self.bridged_last = proposal['proposed_speed_mps'] > 0.\n"
     "        return proposal['proposed_speed_mps'], proposal['steering_rad'], detail\n"),
], PURSUIT: [
    ("            boundary_bridge=bool(self.declare_parameter('g1_boundary_bridge',False).value),\n",
     "            boundary_bridge=bool(self.declare_parameter('g1_boundary_bridge',False).value),\n"
     "            bridge_release_continuity=bool(self.declare_parameter('g1_bridge_release_continuity',False).value),\n"),
]}


def main():
    if CAND.exists():
        shutil.rmtree(CAND)
    shutil.copytree(BASE, CAND)
    chunks = []
    for rel, edits in EDITS.items():
        text = (BASE/rel).read_bytes().decode('utf-8')
        # 작업트리 파일별 줄바꿈(CRLF/LF)을 그대로 유지한다.
        eol = chr(13)+chr(10) if chr(13)+chr(10) in text else chr(10)
        new = text
        for old, rep in ((o.replace(chr(10), eol), r.replace(chr(10), eol)) for o, r in edits):
            assert new.count(old) == 1, (rel, old[:60], new.count(old))
            new = new.replace(old, rep)
        (CAND/rel).write_bytes(new.encode('utf-8'))
        chunks.extend(difflib.unified_diff(text.splitlines(True), new.splitlines(True),
                                           f'a/src/{rel}', f'b/src/{rel}'))
    (HERE/'candidate_A.patch').write_bytes(''.join(chunks).encode('utf-8'))
    print(f'patch lines: {len(chunks)}')


if __name__ == '__main__':
    main()
