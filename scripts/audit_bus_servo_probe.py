#!/usr/bin/env python3
"""수신기 코드를 import하지 않고 저장한 서보 패킷/피드백/설정 비교를 감사한다."""
import argparse
import collections
import json
from pathlib import Path


def audit(root):
    result={}
    for folder in sorted(root.iterdir()):
        if not folder.is_dir() or not (folder/'summary.json').exists(): continue
        summary=json.loads((folder/'summary.json').read_text())
        transactions=[json.loads(line) for line in (folder/'transactions.jsonl').read_text().splitlines()]
        writes=collections.Counter();requests=0;replies=0;received=[];sequences=[];configuration=[]
        for record in transactions:
            request=record.get('request') or '';response=record['reply']
            if 'seq' in response:sequences.append(response['seq'])
            if not request.startswith('Q '):continue
            requests+=1;tx=bytes.fromhex(request[2:]);assert tx[:2]==b'\xff\xff'
            assert len(tx)==tx[3]+4 and sum(tx[2:])%256==255
            assert tx[4] in (1,2,3) and tx[2]<=253
            if tx[4]==3:
                writes[tx[5]]+=1;assert tx[5] in (40,41)
                if tx[5]==40:assert tx[6] in (0,1)
                if tx[5]==41:assert len(tx)==14 and 5<=tx[6]<=20
            rx=bytes.fromhex(response['rx']);assert not response['io_error']
            if rx.startswith(tx):rx=rx[len(tx):]
            if not rx:continue
            assert rx[:2]==b'\xff\xff' and len(rx)==rx[3]+4
            assert sum(rx[2:])%256==255 and rx[2]==tx[2] and rx[4]==0
            replies+=1
            if tx[4]==2:
                payload=rx[5:-1];assert len(payload)==tx[6]
                if tx[5]==0 and tx[6]==40:configuration.append(payload.hex())
                if tx[5]==56 and tx[6]==15:received.append((response['begin_us'],payload))
        assert all(b==a+1 for a,b in zip(sequences,sequences[1:]))
        rows=[]
        if (folder/'feedback.jsonl').exists():
            rows=[json.loads(line) for line in (folder/'feedback.jsonl').read_text().splitlines()]
        assert len(rows)==len(received)==summary['feedback_samples']
        for row,(begin,payload) in zip(rows,received):
            assert row['mcu_begin_us']==begin
            assert row['position_raw']==int.from_bytes(payload[:2],'little')
            assert row['voltage_v']==payload[6]/10 and row['temperature_c']==payload[7]
            assert row['servo_status']==payload[9]==0
        if summary['completed']:
            assert requests==replies and len(configuration)==2
            assert configuration[0]==configuration[1]==summary['config_before_hex']==summary['config_after_hex']
        groups=collections.defaultdict(list)
        for row in rows:groups[row['label']].append(row)
        settled=[]
        for label,group in groups.items():
            if label.startswith('motion_'):
                target=group[-1]['target'];assert target is not None
                centre=summary['motion']['centre'];assert abs(target-centre)<=114
                settled.append(dict(label=label,target=target,last=group[-1]['position_counts'],
                    error_counts=group[-1]['position_counts']-target))
        result[folder.name]=dict(completed=summary['completed'],requests=requests,replies=replies,
            missing_responses=requests-replies,feedback_samples=len(rows),
            writes_by_address=dict(writes),eeprom_write_requests=0,
            configuration_before_after_identical=len(configuration)==2 and configuration[0]==configuration[1],
            sequence_gaps=0,settled_targets=settled,
            max_settled_error_counts=max((abs(x['error_counts']) for x in settled),default=None))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=audit(args.root)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:{a:b for a,b in v.items() if a!='settled_targets'} for k,v in result.items()},indent=2))
