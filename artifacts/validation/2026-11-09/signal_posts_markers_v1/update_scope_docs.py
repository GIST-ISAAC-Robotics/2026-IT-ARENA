"""현재 방향과 시험 결과를 정리한다. 과거 실행 본문은 보존한다."""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
p=ROOT/'docs/DEVELOPMENT_PLAN.md'
s=p.read_text(encoding='utf8').replace(
    '지름길은 P21/P22의 본선 대비 시간 이득과 진입·복귀 비용으로 결정한다.',
    '지름길은 P21만 본선 대비 시간 이득과 진입·복귀 비용으로 결정한다(P22는 11/9 사용자 결정으로 제외).')
p.write_text(s,encoding='utf8',newline='\n')
p=ROOT/'docs/track/V2026_11_06_MIGRATION_2026_11_07.md'
s=p.read_text(encoding='utf8').replace(
    '## 갈림길 비교를 재개할 때의 설계 조건',
    '## 갈림길 비교의 이전 설계 조건\n\n**11/9 후속 결정:** 아래 P22 비교는 당시 조사 근거로 보존한다. P22 진입·출구 구현과 주행은 재개하지 않으며, P21만 후속 검토한다. 마커 누락 시 두 갈림길 앞에서 본선을 유지하는 기능은 계속 대상이다.')
p.write_text(s,encoding='utf8',newline='\n')
