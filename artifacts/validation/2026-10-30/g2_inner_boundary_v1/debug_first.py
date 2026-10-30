"""첫 선택 호출의 후보별 거절 이유를 출력하는 짧은 진단(시뮬레이션 1초)."""
import json
from pathlib import Path
import sys
from unittest.mock import patch

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
import probe_bypass_entry_sequence as probe
from candidate import CorridorGuide,CorridorPolicy

seen=[]


class G(CorridorGuide):
    def evaluate(self,*a,**k):
        out=super().evaluate(*a,**k)
        if out.get('candidates') and len(seen)<3:
            seen.append([{k2:r.get(k2) for k2 in ('offset_m','transition_m','eligible','reason','preview_return_progress_m',
                'preview_point_gap_m','preview_maximum_steering_rad','target_speed_mps','command_clearance_m')}
                for r in out['candidates']]+[out['reason'],out['selected'],out.get('intent_direction')])
        return out


policy=CorridorPolicy(surface=True,plan=True,onset=False)
with patch.object(probe,'EntryGuide',lambda c,e,**k:G(c,e,corridor_policy=policy,**k)):
    probe.rollout(0.,False,.3,True,True,True,True,'combined',17,True,True,False)
print(json.dumps(seen,indent=1))
