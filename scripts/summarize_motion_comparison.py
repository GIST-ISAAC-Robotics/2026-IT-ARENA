#!/usr/bin/env python3
"""보정 실행 원문을 변경하지 않고 비교 표·주행/제동 궤적 그림을 생성한다."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

LABELS = {'snapshot': 'Snapshot matched', 'none': 'Sequential raw',
          'deskew': 'Deskew only', 'shift': 'Shift only', 'both': 'Deskew + shift'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    root = args.directory.resolve()
    summary = json.loads((root/'summary.json').read_text())
    lines = ['# 회전식 LiDAR 보정 비교', '',
             '같은 제어기·차량·10 Hz·500점·500 Hz 원시 광선. 실물 검증·제품 최대속도 측정이 아닙니다.', '',
             '| 조건 | 보정 | 주행·정지 | 센서/보정 | 종료 | 최고 km/h | 최대 중심선 오차 cm | 주행/제동 포함 노면 밖 표본 | 계산 p95 ms |',
             '|---|---|---|---|---|---:|---:|---:|---:|']
    for row in summary['results']:
        m, mm = row.get('metrics', {}), row.get('motion_metrics', {})
        mode = 'snapshot' if row.get('acquisition')=='snapshot_matched' else row.get('compensation')
        error = m.get('tracking_all_active', {}).get('max_abs_centerline_error_m')
        fmt = lambda v, scale=1: '—' if v is None else f'{v*scale:.3f}'
        ok = lambda v: '통과' if v else '미통과'
        lines.append(f"| {row.get('case', row['name'])} | {mode} | {ok(m.get('fixed_speed_tracking_passed'))} | "
            f"{ok(row.get('acquisition_verified') and row.get('motion_verified'))} | {ok(row.get('shutdown_clean'))} | "
            f"{fmt(m.get('peak_longitudinal_speed_kmh'))} | {fmt(error, 100)} | "
            f"{m.get('road_departure_samples_including_stop', '—')} | {fmt(mm.get('processing_p95_ms'))} |")
    for case in summary['cases']:
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        plotted = False
        for row in summary['results']:
            if row.get('case') != case:
                continue
            folder = (root / row['report']).parent
            path = folder/'trace.jsonl'
            if not path.exists():
                continue
            trace = [json.loads(x) for x in path.read_text().splitlines() if x.strip()]
            trace = [x for x in trace if x['elapsed_s'] >= 0]
            controls = [json.loads(x) for x in (folder/'commands.jsonl').read_text().splitlines() if x.strip()]
            controls = [x for x in controls if x['elapsed_s'] >= 0]
            mode = 'snapshot' if row['acquisition']=='snapshot_matched' else row['compensation']
            label = LABELS[mode]
            axes[0].plot([x['elapsed_s'] for x in trace], [x['planar_speed_mps']*3.6 for x in trace], label=label)
            axes[1].plot([x['elapsed_s'] for x in trace], [x['road_clearance_m']*100 for x in trace], label=label)
            axes[2].plot([x['elapsed_s'] for x in controls], [x['steering_rad'] for x in controls], label=label)
            plotted = True
        if plotted:
            for ax in axes:
                ax.grid(alpha=.3)
                ax.axvline(8., color='gray', linestyle=':', label='Planned braking' if ax is axes[0] else None)
            axes[1].axhline(0, color='black', linestyle='--')
            axes[0].set_ylabel('Speed [km/h]')
            axes[1].set_ylabel('Road margin [cm]')
            axes[2].set_ylabel('Steering [rad]')
            axes[2].set_xlabel('Elapsed simulation time [s]')
            axes[0].legend(fontsize=8, ncol=3)
            fig.suptitle(case + ' / tracking and braking (early stop may precede 8 s)')
            fig.tight_layout()
            fig.savefig(root/(case+'.png'), dpi=140)
            lines += ['', f'![{case} 속도·노면 여유·조향 비교]({case}.png)']
        plt.close(fig)
    lines += ['', '그래프는 제동까지 포함합니다. 8초 세로선은 계획된 제동 시각이며 조기 중단은 더 빨리 발생할 수 있습니다.',
              '노면 여유 0 미만은 평가 외형의 노면 이탈입니다. 실물 접촉 센서 판정이나 실제 차량 폭 변경이 아닙니다.',
              '계산 시간은 이 PC·WSL의 벽시계 계측이며 Jetson 실시간 처리 보장이 아닙니다. 내부만은 오래된 기준 좌표를 의도적으로 사용하는 비교 모드입니다.',
              '원문 report·입력·제어·평가 trace·소스 스냅샷을 각 실행 폴더에서 확인하십시오.']
    (root/'RESULTS.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print(root/'RESULTS.md')


if __name__ == '__main__':
    main()
