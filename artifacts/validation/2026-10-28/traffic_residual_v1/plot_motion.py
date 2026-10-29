"""저장 분석 결과의 대조 그림. 센서/제어 결과를 변경하지 않는다."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
BASE=Path(__file__).resolve().parent
data=json.loads((BASE/'motion_audit.json').read_text())['samples']
chain=json.loads((BASE/'anchor_chain_probe.json').read_text())['samples']
fig,ax=plt.subplots(2,1,figsize=(9,7),sharex=True,layout='constrained')
t=[r['reference_s'] for r in data]
ax[0].plot(t,[r['transported_point_error_m'][0]*1000 for r in data],'o-',label='Encoder/gyro transport error (evaluation)')
ax[0].plot([r['reference_s'] for r in chain],[-r['correction_m'][0][0]*1000 for r in chain],'s--',label='Accumulated sensor surface correction')
ax[0].set_ylabel('Distance (mm)');ax[0].legend(fontsize=8)
ax[1].plot(t,[r['pipeline_nearest_m'][0]*1000 for r in data],'o-',label='Original correspondence distance')
ax[1].plot(t,[r['truth_projected_nearest_m'][0]*1000 for r in data],'--',label='Truth projection (diagnosis only)')
ax[1].plot([r['reference_s'] for r in chain],[r['nearest_m'][0]*1000 for r in chain],'s-',label='Sensor-only accumulated correspondence')
ax[1].axhline(20,color='black',linestyle=':',label='Existing point correspondence threshold')
ax[1].set_ylabel('Nearest return distance (mm)');ax[1].set_xlabel('Simulation time (s)');ax[1].legend(fontsize=8)
for a in ax:a.grid(alpha=.25)
fig.suptitle('Residual wall return during braking | diagnostic, not a control change')
fig.savefig(BASE/'motion_comparison.png',dpi=150)
