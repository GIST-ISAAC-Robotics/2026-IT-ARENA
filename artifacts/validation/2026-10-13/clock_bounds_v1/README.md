# C2 왕복 시각 구간 준비 v1

2026-10-13. MCU 왕복 기록을 **합성**해 정규화 JSONL→독립 CLI→구간 결과를 검사했다. 실제 MCU·Jetson·통신·ROS stamp 변환은 연결하지 않았다.

- [단위/CLI 검사](tests.log): 42개 통과. 시험 전 4파일 [스냅샷](source_snapshot.zip) 보존.
- [11조건 보고서](results/report.json): 정상 7조건은 합성 정답 시각 포함, 거절 4조건은 예상대로 오류. 입력·소스 불변.
- `asymmetric` 구간 폭 10.000020 ms, 서로 반대 비대칭 지연을 교차한 `alternating_delays` 2.100010 ms. RTT/2를 편도 지연이나 정확한 보정값으로 채택하지 않는다.
- ±150 ppm, 명시한 timestamp 오차, 2개 순번 누락을 구분했다. 누락을 메우지 않으며 부팅 혼합·시각 역전·가정 밖 10 ms 시계 점프·외삽 초과는 거절했다. 2^60 ns 초과 시각에서도 18 ns 폭을 부동소수점 반올림으로 잃지 않았다.
- 조건별 원본 `.jsonl`, CLI `.log/.report.json`, 별도 [5파일 스냅샷](results/source_snapshot.zip), [실행 환경/명령](reproduction.txt)을 보존했다.

`ready_to_drive`, `ros_stamp_verified`, `clock_mapping_applied`는 항상 false다. ±200 ppm·5초 적합 구간·100 ms 외삽은 합성 시험의 명시 가정이며 MCU 실측 사양/채택값이 아니다. 기록 수신 이전/이후의 인과 관계만 사용하므로 전송 지연·큐 대기 원인도 식별하지 않는다. [설계·실물 의무](../../../../docs/hardware/HARDWARE_CAPTURE_AUDIT_2026_10_13.md#6-왕복-시각으로-구하는-조건부-시간-구간).

전체 회귀의 첫 시도는 [full_regression.log](full_regression.log)에 보존했다. `xacro` import를 비롯한 수집 오류 24개·2 skipped로 중단됐으므로 전체 회귀 성공이 아니다. 환경 진단/수정 재실행 결과는 별도 파일로 추가하며 이 실패를 덮어쓰지 않는다. 최초 실행 wrapper의 PowerShell 변수 확장 때문에 로그 말미 exit/시간 메타데이터는 비어 있다. probe의 실제 도구 exit 0·조건별 보고서와 pytest 본문 수집 실패를 근거로 사용한다.

[환경 진단](environment_diagnosis.log)에서 WSL `/usr/bin/python3`와 Jazzy `xacro` 설치/import를 확인했다. 정확한 첫 실행의 환경 손상 경로는 재현하지 않아 PowerShell 변수 확장이 유일 원인이라고 확정하지 않는다. 환경을 명시한 [재실행 파일](full_regression_retry.sh)은 실제 executable·xacro 경로·sys.path를 기록했고 [전체 회귀](full_regression_retry.log) **1,881개·75.42초·exit 0**을 확인했다. 패키지 설치나 코드 수정으로 통과시킨 것이 아니다. 이후 추가되는 G1 trace 시각 검사 등 후속 변경은 이 결과의 검증 범위에 소급 포함하지 않는다.
