"""완료 스캔마다 현재 확인을 한 번 시작하는 저장 재생 후보. 운영 미연결."""
from pathlib import Path
import sys
A=Path(__file__).resolve().parent;P=A.parent.parent/'2026-11-12'
sys.path[:0]=[str(P/'current_priority_r286_v1'),str(P/'latest_scan_submission_r273_v1')]
from priority import CurrentPriorityProcess
from submission import eligible


class CurrentScanProcess(CurrentPriorityProcess):
    def offer_next_current(self,snapshot,*,now_s):
        if self.closed or self.fault:raise RuntimeError('route process unavailable')
        if self.current.inflight is not None or self.ready is not None:return False
        last=None if self.last_checked_source is None else self.last_checked_source[2]
        if not eligible(snapshot,now_s=now_s,last_reference_s=-float('inf'),last_source=last,policy='fresh'):
            return False
        return super().offer_next_current(snapshot)
