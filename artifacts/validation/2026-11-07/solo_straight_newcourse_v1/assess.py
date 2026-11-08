"""새 코스 r11 기준과 기존 증속 ON의 실제 표본 비교. 공식 랩타임이 아니다."""
import json,math
from pathlib import Path
HERE=Path(__file__).resolve().parent
BASE=HERE.parent/'junction_handoff_v1'
def analyze(folder):
 r=json.loads((folder/'run1/report.json').read_text())
 data=json.loads((folder/'run1/trace.json').read_text())
 active=[q for q in data if q['autonomy'].get('started')]
 t=[q['time'] for q in active]; v=[abs(q['dynamics']['truth_longitudinal_mps']) for q in active]
 dt=[b-a for a,b in zip(t,t[1:])]
 durations={str(k):sum(d for d,x in zip(dt,v) if x>=k) for k in (2.,10/3.6*.95,10/3.6)}
 first=next((i for i,x in enumerate(v) if x>.1),len(v))
 groups=[];start=None
 for i in range(first,len(v)):
  if v[i]<.05:
   if start is None:start=i
  elif start is not None:
   groups.append(dict(first_s=t[start],last_s=t[i-1],span_s=t[i-1]-t[start],progress_m=active[start].get('progress_m')));start=None
 if start is not None:groups.append(dict(first_s=t[start],last_s=t[-1],span_s=t[-1]-t[start],progress_m=active[start].get('progress_m')))
 # t末尾まで丸1秒ある窓だけ。線形補間の積分を用いる。
 def integral(a,b):
  value=0.
  for i in range(len(dt)):
   lo,hi=max(a,t[i]),min(b,t[i+1])
   if hi<=lo:continue
   slope=(v[i+1]-v[i])/dt[i]
   value+=(v[i]+slope*(lo-t[i])+v[i]+slope*(hi-t[i]))*.5*(hi-lo)
  return value
 one=max((integral(a,a+1.) for a in t if a+1.<=t[-1]),default=None)
 return dict(raw_passed=r['passed'],progress_m=r.get('progress_m'),completed_laps=r.get('completed_laps'),
  active_elapsed_s=t[-1]-t[0],peak_mps=max(v),peak_kmh=max(v)*3.6,peak_one_second_mean_mps=one,
  time_over_thresholds_s=durations,below_005_after_motion=groups,max_below_005_span_s=max((q['span_s'] for q in groups),default=0.),
  zero_command_samples=sum(q['autonomy'].get('speed_command_mps')==0 for q in active),
  scope='simulation truth evaluation; first started sample to distance gate, not finish-line timing')
a,b=analyze(BASE),analyze(HERE)
r=dict(baseline=a,candidate=b,elapsed_delta_s=b['active_elapsed_s']-a['active_elapsed_s'],peak_delta_mps=b['peak_mps']-a['peak_mps'])
(HERE/'comparison.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(r,ensure_ascii=False))
