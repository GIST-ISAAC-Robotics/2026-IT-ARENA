"""10 Hz로 같은 앞면이 매번 돌아온다면 기존 추종 속도 근거가 남는지 분리한다. 폐루프 시험 아님."""
import json
from pathlib import Path
from test_bypass_handoff import moving_monitor

BASE=Path(__file__).resolve().parent


if __name__=='__main__':
    out=[]
    for rate in (0.,.25,.35,.50,.55,.65,.825,.85):
        m,states=moving_monitor([rate]*12)
        out.append(dict(lead_speed_mps=rate,step_at_10Hz_m=rate*.1,
            last_forward_rate_mps=m.forward_rate_mps,last_motion_rate_mps=m.motion_rate_mps,
            motion_evidence_scans=sum(s[0] for s in states),max_stationary_history_scans=max(s[1] for s in states)))
    with (BASE/'pace_window.json').open('x',encoding='utf-8') as f:
        json.dump(dict(scope='지면 기준 등속 앞면, ego 0.35 m/s, 10 Hz·매회 같은 조각 공급. G1이나 실제 차량 전체 속도가 아니다.',
                       results=out),f,ensure_ascii=False,indent=2)
    print([(p['lead_speed_mps'],p['last_forward_rate_mps']) for p in out])
