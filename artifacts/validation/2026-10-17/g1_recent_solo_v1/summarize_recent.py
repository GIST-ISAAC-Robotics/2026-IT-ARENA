"""이번 실제 실행의 관측 보완 판단 집계. 새 주행이나 물리 안전 판정은 아니다."""
import argparse
from collections import Counter
import json
from pathlib import Path
import hashlib

def digest(path):
    value=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):value.update(chunk)
    return value.hexdigest()

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('capture',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();assert not args.output.exists()
    before=digest(args.capture)
    reasons=Counter();support_reasons=Counter();rows=[];transitions=[]
    sequence=-1;footer=None;last_reason=None;calls=0;selected=0;first_selected=None
    for line in args.capture.open():
        event=json.loads(line)
        if event['seq']!=sequence+1 or footer is not None:raise ValueError('capture gap/trailing record')
        sequence=event['seq'];data=event['data']
        if sequence==0:
            assert event['kind']=='header' and data['schema']==1
            assert data['follow_config']['recent_observation_enabled'] is True
        elif event['kind']=='footer':footer=data
        elif event['kind']=='call':
            calls+=1
            speed,steering,meta=data['output'];recent=meta.get('recent_observation',{})
            reasons[meta['reason']]+=1
            support_reasons[recent.get('reason','not_evaluated')]+=1
            row=dict(seq=sequence,t_ref_s=data['input']['reference_s'],
                source_first_s=meta.get('source_first_s'),reason=meta['reason'],
                motor_speed_mps=data['input']['speed_mps'],target_mps=speed,
                steering_rad=steering,selected=meta.get('recent_observation_selected',False),
                recent=recent)
            if row['selected']:
                selected+=1
                if first_selected is None:first_selected=row
            if recent.get('required'):rows.append(row)
            if meta['reason']!=last_reason:
                transitions.append({k:v for k,v in row.items() if k!='recent'})
                last_reason=meta['reason']
        elif event['kind']!='reset':raise ValueError('unknown kind')
    assert footer is not None and calls
    assert all(footer.get(k)==0 for k in ('capture_errors','failed','dropped','pending'))
    assert footer.get('worker_alive') is False
    assert footer['written']==sequence and footer['accepted']==sequence
    report=dict(scope='runtime_record_summary_not_new_driving_or_replay',calls=calls,selected=selected,
        reasons=dict(reasons),support_reasons=dict(support_reasons),first_selected=first_selected,
        transitions=transitions,rows=rows,input_sha256=before,input_unchanged=before==digest(args.capture),
        script_sha256=digest(Path(__file__)))
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','transitions')}))

if __name__=='__main__':main()
