# r468 RELEASED 자기 가림 판단 통일

## 원인과 변경

[r465 실제 단독 주행](../../2026-11-13/solo_full_lap_r465_v1/RESULT.md)은9.54m에서8.086sim초 무진행으로 실패했다. 최초 지속정지의 현재 일반 미관측은0, 앞면 가림은0이지만 왼쪽 자기 가림으로 들어가는 예측 외곽11.62mm가1cm 제한을 넘었다. 같은 호출의 `structure_aware_v1`은 실제 점유·숨은 기억을 검사하고 주행을 허용했지만, RELEASED 선택기가 다시 평행 후보로 돌려 `parallel_hold_reference_not_straight`의0을 선택했다. 이후94계획 모두0이며 current 계산 자체가 없었다. command_expired0과 별개인 계획 거절이다.

단독 현재검사에 적용한 구조 그림자 기하를 RELEASED 계획/현재검사에도 연결했다. 같은 현재 원점·광선 방향·양옆 반환·패딩으로 숨을 수 있는 폭을 검사하며 기존5cm 위험 예산을 사용한다. 측방 물체가 현재 있으면 이 경로로 면제하지 않고, 관측 이력·통과 상태·retained의0.35m/s 상한을 유지한다. 가림을 자유 관측으로 채우지 않으며 실제 점유·전방 관측·일반 unknown·숨은 기억·독립 보호의 검사는 남는다. 이1cm 수치는 충돌 거리와 같지 않으며 새로운 차량 간격 완화가 아니다.

구조 검사를 통과한 목표에는 기존의 낮은 속도 재시도 사슬을 반복하지 않는다. 현재검사는 새 입력으로 다시 수행한다. 독립 검토에서 plan/current의 `traffic_recovery_trial` 선택 조건 차이를 발견해 RELEASED current에도 같은 flag를 추가했다. 기존 SOLO 조건은 그대로다. [독립 검토](REVIEW.md).

## 저장 입력과 회귀

원 r465 제출207개를 원순서로 재생해 **base207/207 전체 exact**를 먼저 확인했다. 변경본은113개 exact, **원0→양수94개**, 기존 양수→0/감소0이었다. 측방 상태와 handoff 상태 변경도0이다. 최초 정지 대표3입력은 plan/current에서 각각0.35m/s를 반환했다. 이는 동일한 저장 센서 입력을 양쪽 계산에 넣은 국소 검사이며 새 발행·차량 이동 증거가 아니다. [원본](base_replay_v1/summary.json), [변경본](stage_replay_v1/summary.json).

관련184회귀를 확인했다. 실제 가까운 반환, 선언 밖 누락, 일반 unknown, 앞면 가림, 넓은 그림자, 정책 OFF, 기존 GUARD와 숨은 기억, 현재경로·STOP 경계를 포함한다. 최초 검사에서는 ROS 환경 누락으로 수집 실패했고, 다음에는 격리 stage의 원본 fixture/스크립트 경로가 빠져32개 실패했다. 공통 경로에서183개가 통과했고 scripts import 경로가 빠진CLI1개만 같은 assertion으로 다시 실행해 통과했다. 원 로그를 보존하며 이를 정책 실패나 전체 재실행 성공으로 바꾸어 쓰지 않는다. [183개 결과](integration_pytest_v1.log), [CLI1개 결과](integration_pytest_cli.log).

## 통합과 남은 범위

공통에는 새 `structure_region.py`, `pass_release_trial.py`, `race_runtime_policy/current_region.py`, 관련 시험을 반영했다. 동시에 [r467](../current_structure_cost_r467_v1/RESULT.md)의 판단 exact 수치 최적화를 `known_structure.py`에 통합했다. source hash와 검사/실행 종료는 [integration.json](integration.json)에 남겼다. 실제 주행 조건·센서·물성·속도·정지 판정은 변경하지 않았다.

가림 안의 작은 물체와 동적 접근, 실물 장착/센서 오차는 남는다. r467 최적화 후에도 ARM 대표 현재검사 최대132.37ms와 기존 heavy GUARD의100ms 초과가 남아 별도 수치 최적화를 진행한다. 실제 전체 ROS10Hz나 실차 완료로 취급하지 않는다. 다음은 같은 조건의 r469 단독1회이며, 기존 r465 실패와 원시 증거는 보존한다.
