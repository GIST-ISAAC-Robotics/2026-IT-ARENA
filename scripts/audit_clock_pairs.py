#!/usr/bin/env python3
"""호스트 단조/ROS 시각의 이중 읽기 기록 감사. timestamp 변환/보정은 하지 않는다."""
import argparse
import hashlib
import json
from pathlib import Path

from arena_vehicle_interface import clock_pair_audit
from arena_vehicle_interface.clock_pair_audit import ClockPair, audit_pairs, epoch
from audit_clock_exchanges import exact_keys, unique_object, invalid_constant

SCHEMA='arena_clock_pair_audit_v1'
MAX_BYTES=8_000_000


def audit_bytes(data):
    if len(data)>MAX_BYTES:raise ValueError('capture exceeds byte budget')
    lines=data.decode('utf-8').splitlines()
    if not 4<=len(lines)<=10002 or any(not x.strip() for x in lines):
        raise ValueError('header, two through 10000 pairs, footer required')
    rows=[json.loads(line,object_pairs_hook=unique_object,parse_constant=invalid_constant) for line in lines]
    header,footer=rows[0],rows[-1]
    exact_keys(header,('kind','schema','origin','host_clock','host_epoch','ros_epoch','source_context'))
    if (header['kind']!='header' or header['schema']!=SCHEMA or
            header['host_clock']!='time.monotonic_ns' or
            header['origin'] not in ('synthetic','pc_clock_probe','hardware_capture')):
        raise ValueError('explicit schema, origin, monotonic clock required')
    epoch(header['host_epoch'])
    epoch(header['ros_epoch'])
    if not isinstance(header['source_context'],str) or not 1<=len(header['source_context'])<=1000:
        raise ValueError('bounded source context required')
    exact_keys(footer,('kind','completed','expected_count','written_count'))
    if footer['kind']!='footer' or type(footer['written_count']) is not int or footer['written_count']!=len(rows)-2:
        raise ValueError('footer row count mismatch')
    fields=tuple(ClockPair.__dataclass_fields__)
    pairs=[]
    for row in rows[1:-1]:
        exact_keys(row,('kind',*fields))
        if row['kind']!='pair':raise ValueError('pair rows required')
        pairs.append(ClockPair(**{k:row[k] for k in fields}))
    result=audit_pairs(pairs,declared_host_epoch=header['host_epoch'],declared_ros_epoch=header['ros_epoch'],
                       completed=footer['completed'],expected_count=footer['expected_count'])
    return {**result,'schema':SCHEMA,'origin':header['origin'],'source_context':header['source_context']}


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists() or args.input.resolve()==args.output.resolve():parser.error('new output required')
    with args.input.open('rb') as stream:data=stream.read(MAX_BYTES+1)
    before=hashlib.sha256(data).hexdigest()
    try:result=audit_bytes(data)
    except (ValueError,TypeError,UnicodeError) as error:
        result=dict(record_integrity_valid=False,error=str(error),clock_mapping_applied=False,
                    ready_to_drive=False,ros_stamp_verified=False)
    result.update(input_sha256=before,input_unchanged=before==digest(args.input),
                  source_sha256={Path(p).name:digest(p) for p in (__file__,clock_pair_audit.__file__)})
    if not result['input_unchanged']:result['record_integrity_valid']=False
    with args.output.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,indent=2,ensure_ascii=False,allow_nan=False)
        stream.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('elapsed_intervals','source_sha256')},ensure_ascii=False))
    return 0 if result['record_integrity_valid'] else 2


if __name__=='__main__':raise SystemExit(main())
