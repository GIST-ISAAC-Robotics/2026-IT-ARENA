# 원천 입력 나이의 MCU 단위 환산 후보

- [실행 스크립트](run_checks.sh), [결과](tests.log): 관련 **291 passed·0 skipped**, [exit 0](checks_exit.txt), [소스 해시 차이](source_hash_diff.txt) 0 B. 당시 소스는 `source_snapshot.tar.gz`에 보존했다.
- host/MCU 시계 비율 범위에서 나이는 올림·TTL은 내림하여 환산한다. 미래/오래된 입력·ARM 이전 표본·epoch 변경·정수 분해능·큰 host offset·불확실성 확대를 검사했다.
- 선언한 ±200 ppm 범위의 합성 참 시계와 원천 시각 오차를 대조하여, 채택한 시간 후보의 수신기 만료가 원래 입력의 TTL을 넘지 않는지 확인했다. 이 값은 실물 oscillator 측정값이 아니다.
- **모터축 28카운트·15:1·65 mm**의 기존 Python 수신기가 같은 후보 `source_age_us`/`ttl_us`를 받으면 같은 만료를 계산하는 것을 확인했다. 패킷 대입은 시험 fixture에서만 수행했다.
- 후보 자체는 명령을 생성/송신하지 않는다. ROS→host 변환·원천 watermark·ARM ACK 상관 확인·실제 시계/지연은 별도 검증 대상이다. `ready_to_drive=false`, `runtime_connected=false`.
- [공식과 해석 범위](../../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#23-1023-후속--원천-입력-나이의-mcu-단위-환산-후보).
