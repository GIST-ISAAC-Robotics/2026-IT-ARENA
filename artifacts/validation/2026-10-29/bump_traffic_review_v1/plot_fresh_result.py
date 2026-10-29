"""완료된 방지턱 교통의 연속 확인/같은 스캔 일시 대기 연결을 비교한다."""
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE=Path(__file__).resolve().parent
BUMP = BASE.parent.parent/'2026-10-28/bump_traffic_v1'
RUNS={'Rolling confirmation run3':BUMP/'run3', 'Fresh-scan wait run4':BUMP/'run4'}


def main():
    image=BASE/'fresh_comparison.png'
    metadata=BASE/'fresh_comparison.json'
    if image.exists() or metadata.exists():
        raise FileExistsError('comparison already preserved')
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True,layout='constrained')
    results={}
    for label,path in RUNS.items():
        report=json.loads((path/'report.json').read_text(encoding='utf-8'))
        if not report.get('shutdown_clean'):
            raise ValueError('requires completed run with shutdown result')
        source=path/'trajectory.jsonl'
        raw=source.read_bytes()
        records=[json.loads(line) for line in raw.decode('utf-8').splitlines()]
        first=report['case_window']['green_sim_s'];last=report['case_window']['end_sim_s']
        rows=[r for r in records if first<=r['t']<=last]
        if not rows: raise ValueError('no driving samples')
        origin=rows[0]['progress_m']
        times=[r['t']-first for r in rows]
        axes[0].plot(times,[r['progress_m']-origin for r in rows],label=label)
        axes[1].plot(times,[r['drive_speed_mps'] for r in rows],label=label,alpha=.8)
        axes[2].plot(times,[r['ground_speed_mps'] for r in rows],label=label,alpha=.8)
        results[label]=dict(trajectory_sha256=hashlib.sha256(raw).hexdigest(),
            original_passed=report['passed'],run_valid=report['run_valid'],
            failed_gates=[k for k,v in report['gates'].items() if not v],
            samples=len(rows),sampled_net_progress_m=rows[-1]['progress_m']-origin,
            dense_progress_m=report['evaluation']['full_course_progress']['progress_m'])
        assert source.read_bytes()==raw
    axes[0].axhline(43.39333,color='grey',ls=':',lw=1,label='Previous stall position')
    for axis,label in zip(axes,['Progress (m)','Planner command (m/s)','Measured body speed (m/s)']):
        axis.set_ylabel(label);axis.grid(alpha=.25);axis.legend(loc='upper left')
    axes[-1].set_xlabel('Simulation seconds after applied green')
    fig.suptitle('Official signal + traffic | rolling confirmation vs fresh-scan wait\nOne run each; bump ON; no hardware claim')
    fig.savefig(image,dpi=150)
    metadata.write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(results))


if __name__=='__main__':
    main()
