"""C4의 부분 진전과 남은 실패, 실행 환경 복구를 기록한다."""
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
p=A/'README.md';s=p.read_text(encoding='utf8')
s+='''
## C4: 본선에는 돌아왔으나 복귀 완료·주행 지속 미달

r58과 좁은 가림체 선택 ON(실제 header 확인)으로 같은 정지차 장면을 1회 실행했다. 진행2.536m, 상대통과11.106초, 마지막2초 차체 전체 도로 안20표본을 확인했다. 그러나 최종BYPASS·최장정체6.470초·복귀후진행 미달로 **원report와 보충목표 모두 실패**다. C3보다 수치가 나아졌다는 사실을 완료로 쓰지 않는다.

최소차량간격0.122875m/보간0.122869m·접촉/겹침0, 최대두바퀴잔디·잔디12.424초, STOP변위0.0000289m·정착0.5초·정상종료·소스불변이다. 실제 runtime 지주/발판 최소0.161698/0.143250m·겹침0이지만 신호 설치 y를 지나지 못했다. 최종 중앙편이0.13686m로 본선 중앙 정렬이 완료된 것은 아니다.

`c4_reason_summary.json`에서 23.42초부터 `entry_guide_current_wall_too_short`가 재발했고 24.52초 이후 지속됐다. 전체 해당분기는198판단, 이외 스캔후반 반복 `no_entry_command_candidate`가115판단이다. C3의 지주 가림 연결은 실제 경로를 바꾸었지만 모든 벽지지/잔여 점유 문제를 해결하지 않았다. 저장 입력의 후반 현재 반환 묶음과 실제 벽지지를 추가로 분리한다.

**실행 환경:** C4 종료·잔여프로세스0 확인 뒤 새 저장재생 명령이 Python 시작 전 `Wsl/Service/E_UNEXPECTED`로 실패했다. `/bin/true`도 같은 오류라 Ubuntu-24.04 배포판만 terminate/restart했고 `WSL_RECOVERED` 종료0을 확인했다. Windows·전원설정은 변경하지 않았다. 원실패 `checks/c4_wall.log/.exit`와 재시도 `c4_wall_retry1`을 구분한다. 시뮬레이션 실패를 재실행한 것이 아니다.
'''
p.write_text(s,encoding='utf8')
p=R/'docs/PROJECT_CONTEXT.md';s=p.read_text(encoding='utf8')
s=s.replace('관련191+49검사 통과 후 **C4 실제1회 검증 중**이다.',
    '관련191+49검사 뒤 **C4는2.536m·본선 복귀 표본까지 확인했으나 최종BYPASS·최장정체6.470초로 실패**했다. 접촉0·최대두바퀴잔디·STOP/종료·소스불변이며, 복귀 말기의 벽 지지/점유 분기를 저장 입력으로 분리한다.')
p.write_text(s,encoding='utf8')
p=R/'docs/track/V2026_11_06_MIGRATION_2026_11_07.md';s=p.read_text(encoding='utf8')
s=s.replace('관련191+49검사 뒤 C4를 같은장면에서1회 검증 중이다.',
    '관련191+49검사 뒤 C4는2.536m·본선 복귀 표본을 확인했지만 최종BYPASS·6.470초정체로 실패했다. 말기 벽지지 부족을 저장 입력으로 분리한다.')
p.write_text(s,encoding='utf8')
print('C4 result and WSL failure/recovery recorded')
