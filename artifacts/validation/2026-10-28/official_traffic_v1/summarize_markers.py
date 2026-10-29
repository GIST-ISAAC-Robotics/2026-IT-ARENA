"""보존된 상태 기록의 마커 노출/벽 선택 변화. 프레임 검출률이나 인식 거리 시험이 아니다."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def summarize(run):
    path=run/'autonomy_status.jsonl'
    before=sha(path)
    counts=Counter(); intervals=[]; side_changes=[]; previous_side=None; last=None
    rows=0; backward=0; last_time=None
    for line in path.open(encoding='utf-8'):
        d=json.loads(line);t=d.get('sim_time_s')
        if not isinstance(t,(float,int)) or not math.isfinite(t):continue
        rows+=1
        if last_time is not None and t<last_time:backward+=1
        last_time=t
        ids=tuple(sorted(set(d.get('marker_ids') or [])))
        counts.update(ids)
        if ids:
            if last is not None and last['ids']==list(ids) and 0<=t-last['end_s']<=.5:
                last['end_s']=t;last['status_samples']+=1
            else:
                last=dict(ids=list(ids),start_s=t,end_s=t,status_samples=1)
                intervals.append(last)
        else:last=None
        side=d.get('side')
        if side in ('left','right'):
            if previous_side is not None and side!=previous_side:
                side_changes.append(dict(t_s=t,previous=previous_side,selected=side,ids=list(ids)))
            previous_side=side
    return dict(scope='autonomy status samples; repeated detections may be the same camera frame; not frame recall or ground-truth marker visibility',
        input_sha256=before,input_unchanged=sha(path)==before,status_rows=rows,
        time_reversals=backward,marker_status_counts=dict(counts),intervals=intervals,side_changes=side_changes)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=summarize(a.run)
    with a.output.open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in ('intervals','input_sha256')}))
