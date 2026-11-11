"""r152 범위 끝 결측의 원시 360도 이웃. 거리나 판단을 변경하지 않는다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import math
import numpy as np
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.scan_input import scan_angles, scan_cell_widths

A=Path(__file__).resolve().parent
TRACE=A.parent/'adjacent_live_r152_v1/two_actor_run2/run1/follow_decisions.jsonl'

def main():
    digest=hashlib.sha256(TRACE.read_bytes()).hexdigest()
    events=[json.loads(s) for s in TRACE.open()]
    _,config=make_controller(events[0]['data']);rows=[]
    for event in events:
        if event['kind']!='call':continue
        normal=event['data']['output'][2].get('normal',event['data']['output'][2])
        partial=normal.get('partial_observation',{})
        if partial.get('reason')!='unbounded_missing':continue
        data,scan,hist=restore_call(event['data']['input'],config)
        ranges=np.asarray(scan.ranges);angles=(scan_angles(scan)+config.lidar_yaw_rad+math.pi)%math.tau-math.pi
        order=np.argsort(angles);front=order[np.abs(angles[order])<=math.radians(100)+1e-6]
        valid=np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<=scan.range_max)
        cells=scan_cell_widths(scan);ends=[]
        for edge in (front[0],front[-1]):
            if valid[edge]:continue
            k=int(np.flatnonzero(order==edge)[0]);lo=hi=k
            while lo>0 and not valid[order[lo-1]]:lo-=1
            while hi<len(order)-1 and not valid[order[hi+1]]:hi+=1
            ids=order[lo:hi+1];neighbors=order[max(0,lo-2):min(len(order),hi+3)]
            ends.append(dict(edge_index=int(edge),component_indices=ids.tolist(),
                component_width_deg=math.degrees(float(sum(cells[ids]))),
                bracketed=lo>0 and hi<len(order)-1,
                cells=[dict(index=int(i),angle_deg=math.degrees(float(angles[i])),range_m=float(ranges[i]),
                            valid=bool(valid[i]),front=bool(i in front)) for i in neighbors]))
        rows.append(dict(seq=event['seq'],reference_s=data['reference_s'],speed=data['speed_mps'],
            reason=normal.get('reason'),partial=partial,ends=ends))
    (A/'edge_details.jsonl').write_text(''.join(json.dumps(freeze(r))+'\n' for r in rows))
    summary=dict(calls=len(rows),ends=sum(len(r['ends']) for r in rows),
        all_ends_bracketed=all(e['bracketed'] for r in rows for e in r['ends']),
        widths_deg=sorted(set(round(e['component_width_deg'],6) for r in rows for e in r['ends'])),
        trace_sha256=digest,trace_unchanged=hashlib.sha256(TRACE.read_bytes()).hexdigest()==digest,
        representative=next((r for r in rows if r['seq']==635),None))
    (A/'inspection.json').write_text(json.dumps(freeze(summary),indent=2))
    print(json.dumps(freeze(summary)))

if __name__=='__main__':main()
