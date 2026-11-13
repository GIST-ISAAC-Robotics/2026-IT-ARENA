# ROS 배열 비교 수정 — r394 v2

원 `verify_ros_pty_v1`은 통합 전체 실패로 보존한다. 실제 CLEAR/보류ARM/DRIVE·원 source·종료 STOP tx/ACK는 통과했지만 `m.velocity == [0.]`는 ROS의 `array.array` 표현을 Python list와 직접 비교했다. 이는 항목값 동등 비교가 아니다. 이 항목은 아직 원 값 기록이 없어 성공으로 소급하지 않는다.

후속 검사기만 `list(m.velocity) == [0.]`로 바꾸고 원 수신 stamp/name/frame/position/velocity와 컨테이너 종류를 보고서에 저장한다. 호스트·lease·정지 판단·시각·제어기 코드는 바꾸지 않는다. 순수106검사는 이미 통과했으므로 반복하지 않고 새 PTY 통합 한 번만 진행한다. 최대15초·domain217·실물 포트 없음 조건을 유지한다.
