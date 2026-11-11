"""발행/수신 행을 원래 stamp로 대조한다. 센서 노출시각 보장이 아니다."""
import csv, hashlib, json
from collections import Counter
from pathlib import Path

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
report={}
for run in (2,3):
    sent_path=ROOT/f'build/r94_run{run}_sent.csv'
    received_path=ROOT/f'build/r94_run{run}_received.csv'
    with sent_path.open() as f:sent=list(csv.DictReader(f))
    with received_path.open() as f:received=list(csv.DictReader(f))
    stamps=[int(r['stamp_ns']) for r in sent]
    received_stamps=[int(r['stamp_ns']) for r in received]
    sent_counts,received_counts=Counter(stamps),Counter(received_stamps)
    by_stamp={int(r['stamp_ns']):r for r in sent}
    missing=[i for i,s in enumerate(stamps) if s not in received_counts]
    extra=[s for s in received_stamps if s not in sent_counts]
    mismatch=[i for i,r in enumerate(received) if int(r['stamp_ns']) in by_stamp and
              any(r[k]!=by_stamp[int(r['stamp_ns'])][k] for k in ('bytes','crc32'))]
    report[str(run)]=dict(sent=len(sent),received=len(received),missing_sent_indices=missing,
        extra_received=len(extra),mismatch_rows=mismatch,
        repeated_sent=sum(n-1 for n in sent_counts.values()),repeated_received=sum(n-1 for n in received_counts.values()),
        received_stamp_reversals=sum(b<=a for a,b in zip(received_stamps,received_stamps[1:])),
        negative_receive_age=sum(float(r['elapsed_ms'])<0 for r in received),
        sent_sha256=sha(sent_path),received_sha256=sha(received_path),
        missing_relative_time_s=[(stamps[i]-stamps[0])/1e9 for i in missing])
output=A/'delivery_audit.json'
assert not output.exists()
output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
print(json.dumps(report))
