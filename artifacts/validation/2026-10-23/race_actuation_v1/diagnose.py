import json
from test_race_actuation_contract import CoupledBench

b=CoupledBench()
b.frame(0)
try:
    b.frame(1)
except AssertionError:
    now=b.now_us/1e6
    msg=b.image(1)
    header=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    report=dict(now_repr=repr(now),header_repr=repr(header),difference_s=now-header,
                vision=b.vision.snapshot(),second_frame_returned=False)
else:
    report=dict(second_frame_returned=True)
with open('artifacts/validation/2026-10-23/race_actuation_v1/diagnosis.json','x') as out:
    json.dump(report,out,indent=2)
print(json.dumps(report))
