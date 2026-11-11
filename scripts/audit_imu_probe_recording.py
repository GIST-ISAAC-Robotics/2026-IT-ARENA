#!/usr/bin/env python3
"""수신기 모듈을 import하지 않고 UART 원문/JSONL/집계의 일치를 검사한다."""
import argparse
from collections import Counter
import json
from pathlib import Path
import struct
import zlib


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    args=parser.parse_args()
    raw=(args.capture/'uart.bin').read_bytes()
    frames=[]; offset=0; crc_errors=0; bad_headers=0
    while True:
        at=raw.find(b'IMU1',offset)
        if at<0 or at+68>len(raw): break
        offset=at+1
        if struct.unpack_from('<HBB',raw,at+4)[:2]!=(68,1):
            bad_headers+=1; continue
        packet=raw[at:at+68]
        if zlib.crc32(packet[:64])!=struct.unpack_from('<I',packet,64)[0]:
            crc_errors+=1; continue
        frames.append(packet); offset=at+68
    kinds=Counter(); gaps=[]; prev=None; first_measured=None; last_measured=None
    worst_receipt_ms=0.; measured_clock_pairs=[]; count=0
    with (args.capture/'frames.jsonl').open() as src:
        for packet,line in zip(frames,src,strict=True):
            r=json.loads(line); count+=1
            boot,seq=struct.unpack_from('<II',packet,8)
            begin,end=struct.unpack_from('<QQ',packet,16)
            assert (boot,seq,begin,end)==(r['boot'],r['sequence'],r['read_begin_us'],r['read_end_us'])
            assert packet[40:56].hex()==r['payload_hex']
            assert packet[7]==r['type'] and packet[39]==r['tag_raw']
            if prev and boot==prev['boot']:
                jump=(seq-prev['sequence'])&0xffffffff
                if jump!=1:gaps.append(jump-1)
                if r['phase']=='measure' and prev['phase']=='measure':
                    worst_receipt_ms=max(worst_receipt_ms,(r['host_monotonic_ns']-prev['host_monotonic_ns'])/1e6)
            if r['phase']=='measure':
                if first_measured is None:first_measured=r
                last_measured=r
                if packet[7]==1:
                    kind=packet[39]>>3; kinds[str(kind)]+=1
                    if kind==4:
                        measured_clock_pairs.append((struct.unpack_from('<I',packet,40)[0],begin))
            prev=r
    summary=json.loads((args.capture/'summary.json').read_text())
    assert dict(kinds)==summary['raw_tags']
    assert crc_errors==summary['decoder'].get('crc_errors',0)
    assert bad_headers==summary['decoder'].get('invalid_header',0)
    assert gaps==[g['jump']-1 for g in summary['sequence_gaps']]
    tick=None
    if len(measured_clock_pairs)>1:
        (s0,m0),(s1,m1)=measured_clock_pairs[0],measured_clock_pairs[-1]
        tick=(m1-m0)/((s1-s0)&0xffffffff)
    result=dict(raw_json_summary_consistent=True, wire_bytes=len(raw),decoded_frames=count,
                measured_tag_counts=dict(kinds),crc_errors=crc_errors,bad_headers=bad_headers,
                missing_sequence_frames=sum(gaps),sequence_gap_sections=len(gaps),
                measured_max_receipt_gap_ms=worst_receipt_ms,
                observed_sensor_tick_us_against_mcu_read_span=tick,
                time_contract_verified=False)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
