# M4 시각 판정 후속

run61에서 자차 pose 수신 시각 7.619 s와 MCU 원본 시각 7.620 s를 비교해 마지막 등록 STOP 표본 하나를 잘못 제외했다. 실제 MCU 스트림에서는 같은 7.620 s에 generation 2 `software_stop` 뒤 generation 3 `explicit_clear`가 순서대로 기록됐다.

수정은 같은 MCU 스트림의 원본 시각·boot·generation·순서로 CLEAR 경계를 정한다. 시간 여유를 넓혀 모든 STOP을 허용하지 않는다. CLEAR 이후 옛 STOP, 다른 세대, 예상 밖 FAULT/STOP, CLEAR 근거 누락은 계속 거절한다. 같은 us 전이를 재현하는 회귀를 추가했고 관련 135개를 통과했다.

[run61 사후 재평가](run61_reassessment.json)는 등록 STOP 조건을 통과했다. 다만 기존 앞차 HOLD 1초 기준에 0.912초만 머물렀으므로 종합 `passed=false`를 유지했다. 원본 보고서·원시 입력 해시가 불변임을 확인했다. 새 주행 결과나 원래 실패의 소급 변경이 아니다.