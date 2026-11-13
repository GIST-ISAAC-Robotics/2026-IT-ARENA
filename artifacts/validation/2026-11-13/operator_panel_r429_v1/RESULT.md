# r429 현재 검증 범위

## 완료

- 공통 원본 SHA 불변. stage 변경8파일의 Python AST 구문 확인.
- Windows 기존 운영/jog/속도상한/경기/수동출발140검사 통과.
- Windows 새 경계19검사 통과: 기본 B3 상한·옛 초기값 호환, 각도/v×t 명세, 자율 수동 차단, stale 요청/STOP 큐 취소, 상한 초과 거절, child 재사용/종료, 준비 CLEAR→ARM→실행→STOP 합성 연결, 자율 publisher 차단, 준비 뒤 연결 소실, 서비스 예외 회수, stdin EOF 취소.
- Node DOM 검사 통과: 두 단위 양방향 편집, 서버 적용값 분리, 상태 갱신 시 미적용값 보존, 자율 수동 조작 비활성, 초과 입력을 줄여 실행하지 않음.
- 요청 실행부가 없는 UI 모형이 아니다. 수동 API→ROS 서비스→기존 jog subprocess→`/drive`→guard→STOP 경로를 stage에 구현했다.
- 실제 ROS·HTTP 대표는 새 WSL root-c에서 통과했다. 패널 상한0.15 m/s 적용→명시 CLEAR/ARM→목표0.1 m/s·0.4초·중앙 등가각+5°→`/drive`9개·실제 LidarSafety 양수20개→최종0목표→가짜 MCU STOP ACK/새 `ESTOP_LATCHED`→자식 정상 회수. 실행기 오류0·잔존 프로세스0·동결223파일 변경0이다. `ros_it-arena-r429-panel-c/result_v2/result.json` 및 `ros_source_audit.json`에 보존했다.
- 최초 root-b는 parent 실행중 graph가 새 child endpoint를 다른 publisher로 판단해 시작 전 취소했다. 양수/0 `/drive` 명령0, 가짜 MCU STOP·자식 회수는 완료했다. DDS endpoint의 구체 이름은 원 기록에 없어 원인을 그 이상 단정하지 않는다. 원 실패를 보존하고 실행중 graph 검사는 실제 발행자인 기존 jog의 매 송신 전 검사에 일원화했다. parent의 준비/실행 수락 전 검사와 client/timeout 회수는 유지한다. 이 변경 관련56검사를 통과한 뒤 root-c를 실행했다.
- 첫 PowerShell→bash pipe 실행은 CR 문자 때문에 redirect 구문 오류로 ROS 실행 전 종료했다. root-a의 빈 로그를 보존했고 후속은 LF shell 파일로 실행했다.

## 실행 대기 / 한계

- 아직 공통 원본 이관·설치하지 않았다. 원본SHA/변경목록/diff는 `original_sha256.json`, `changes.json`, `changes.patch`이다.
- 실물 방향·각도·속도·제동·물리 STOP·무선 통신은 검증하지 않았다. 실제 장치/Jetson/flash 접근은 없었다.
- 준비/실행 구분과 기존 0.2m/s·2초 단발 한계를 유지했다. 후진·WASD·자동 자율/수동 프로세스 교체는 없다.
