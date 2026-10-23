# 별도 시계 offer 시간 조건 — 오프라인 후보

- [실행 스크립트](run_checks.sh), [회귀 결과](tests.log): 관련 **217 passed**, [exit 0](checks_exit.txt), [소스 해시 차이](source_hash_diff.txt) 0 B.
- [감사 결과](audit/report.json): 합성 10조건 모두 기대 판정. 비대칭 왕복·늦은 offer·송신 지연 80 ms·넓은 불확실성·주파수 오차·정확한 만료 경계·MCU 재부팅·세대 변경·host 재시작·외삽 초과를 포함한다. 거절이 정답인 사례를 주행 성공으로 세지 않는다.
- 수신 시각부터 임대 수명을 다시 시작하지 않는다. MCU 발행/만료를 host 단조 시각의 구간으로 옮기며 가장 이른 만료보다 도착 상한이 엄격히 앞서야 한다. 중앙값을 채택하지 않는다.
- 100 ms 임대·주파수/취득 시각/편도 지연 상한은 명시한 **합성 가정**이지 실측값이나 운영 설정이 아니다. `/drive/safe` 원천 시각·호스트 세션 상태·구동 허가는 별도다.
- `runtime_connected=false`, `ready_to_drive=false`, `firmware_verified=false`. 운영 ROS/명령 생산기·UART/보드에는 연결하지 않았다.
- 검사 소스는 `source_snapshot.tar.gz`와 감사 하위 `source_snapshot.zip`에 보존했다. [구동 계약 §21](../../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#21-후속-설계--별도-시계의-offer를-호스트에서-사용하기-전).
