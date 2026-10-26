"""저장 trace의 속도/간격 그림. sim-time 기준이며 원래 판정은 변경하지 않는다."""
import hashlib
import json
from pathlib import Path
import argparse
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--narrow',action='store_true')
    args=parser.parse_args()
    assert not args.output.exists()
    root=Path(__file__).resolve().parent
    runs=[('run55_faster_lead_stop','Nominal / 15 cm centered','#2563eb'),
          ('run57_faster_lead_sensor_stress','Noise + delay / 15 cm centered','#e37b16')]
    if args.narrow:
        runs.append(('run59_narrow_offset_lead','Noise + delay / 10 cm, +8 cm offset','#299561'))
    fig,axes=plt.subplots(2,1,figsize=(10,6.5),sharex=True,layout='constrained')
    evidence={}
    for folder,label,color in runs:
        path=root/folder/'run'
        trace_file=path/'trace.json'
        raw=trace_file.read_bytes()
        report=json.loads((path/'report.json').read_text(encoding='utf8'))
        green=report['evaluation']['t_green']
        rows=[r for r in json.loads(raw) if 0<=r['t']-green<=15]
        t=[r['t']-green for r in rows]
        axes[0].plot(t,[r['ground_speed_mps'] for r in rows],color=color,label=label,lw=1.5)
        axes[1].plot(t,[r['sampled_gap_m'] for r in rows],color=color,lw=1.5)
        evidence[folder]={'trace_sha256':hashlib.sha256(raw).hexdigest(),
                          'original_passed':report['passed'],'run_valid':report['run_valid'],
                          'trace_unchanged':hashlib.sha256(trace_file.read_bytes()).hexdigest()==hashlib.sha256(raw).hexdigest()}
    for ax in axes:
        ax.axvspan(4,9,color='#ced4dc',alpha=.45,label='Lead stopped (script)')
        ax.axvline(4,color='#607080',ls=':',lw=1)
        ax.axvline(9,color='#607080',ls=':',lw=1)
        ax.grid(alpha=.2)
        ax.spines[['top','right']].set_visible(False)
    axes[0].set_ylabel('Ego ground speed [m/s]')
    axes[1].set_ylabel('Sampled body gap [m]')
    axes[1].set_xlabel('Simulation time after green [s]')
    axes[1].axhline(.15,color='#b91c1c',ls='--',lw=1,label='Registered minimum: 0.15 m')
    axes[0].legend(loc='upper right',fontsize=8)
    axes[1].legend(loc='upper right',fontsize=8)
    axes[0].set_ylim(bottom=0)
    axes[1].set_ylim(bottom=0)
    axes[1].set_xlim(0,15)
    fig.suptitle('M2 stop / restart — bounded Gazebo scenarios',fontsize=14)
    fig.text(.5,-.025,'Closed-loop runs have different inputs and trajectories. Not a hardware or high-speed braking proof.',ha='center',fontsize=9)
    fig.savefig(args.output,dpi=170,bbox_inches='tight')
    out=args.output.with_suffix('.json')
    assert not out.exists()
    out.write_text(json.dumps({'scope':'post-hoc 10 Hz trace comparison; no original verdict changes',
         'runs':evidence},ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(args.output)


if __name__=='__main__':
    main()
