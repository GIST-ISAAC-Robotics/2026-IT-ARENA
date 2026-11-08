import json
from pathlib import Path
import cv2
import numpy as np
from candidate import MarkerAnchoredStartSignal,detections,late_green
HERE=Path(__file__).resolve().parent
SOURCE=HERE.parent/'continuous_confirmation_v1/run1'
results={}
for name in ('signal_off','signal_red','signal_green'):
    rgb=cv2.cvtColor(cv2.imread(str(SOURCE/(name+'.png'))),cv2.COLOR_BGR2RGB)
    markers=detections(rgb)
    s=MarkerAnchoredStartSignal()
    output=[s.update(rgb,1.+i*.1,markers) for i in range(3)]
    bank,reason=late_green(rgb,markers)
    results[name]=dict(markers=[(i,c.tolist()) for i,c in markers],bank=bank,reason=reason,output=output,state=s.observed)
    if name=='signal_green':green=rgb;anchors=markers
# Distinct actual images are not available here; repeated saved RGB is detector-only evidence.
cases={}
for name,markers in [('missing',[]),('one',[v for v in anchors if v[0]==0]),
                     ('swapped',[(45 if i==0 else 0 if i==45 else i,c) for i,c in anchors])]:
    s=MarkerAnchoredStartSignal();cases[name]=[s.update(green,2.+i*.1,markers) for i in range(4)]
s=MarkerAnchoredStartSignal();repeat=[s.update(green,3.,anchors) for _ in range(8)]
cases['same_stamp']=repeat
s=MarkerAnchoredStartSignal();seq=[s.update(green,4.,anchors),s.update(np.zeros((4,4)),4.1,anchors),s.update(green,4.2,anchors),s.update(green,4.3,anchors)]
cases['invalid_image_interrupt']=seq
(HERE/'probe.json').write_text(json.dumps(dict(actual_saved_rgb=results,counterexamples=cases),indent=2))
print(json.dumps({k:{a:v[a] for a in ('reason','output','state')} for k,v in results.items()}))
assert results['signal_green']['output']==[False,False,True]
assert not any(results['signal_off']['output']) and not any(results['signal_red']['output'])
assert not any(any(v) for v in cases.values())
