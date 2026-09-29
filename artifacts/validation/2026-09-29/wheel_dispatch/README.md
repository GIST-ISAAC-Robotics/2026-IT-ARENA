# 엔코더 전달 지연 재분석과 교차 비교

상태: **ROS/Jazzy·Jetson/Humble 실행 비교 완료, 지연 문제 미해결**. 준비 당시 첫 SSH는 승인 서비스 한도로 실행 전에 차단됐으나 재개 후 접속·실행은 정상 처리됐다. 기본 executor는 `legacy`를 유지한다.

- `prior_abcd_analysis_v1.json`: 기존 영상 분리 A~D의 엔코더 지연·계산 시간·시각 역전 재집계.
- `prior_abcd_analysis_v2.json`: 같은 결과에 최장 지연 표본과 원본 기록기 수신 시각 대조를 추가. 원자료 변경 없음.
- `continuity_prior_check.json`: 새 연속성 판정 함수를 이전 결과에 대입. A/B 참, C/D 거짓이며 옛 보고서는 수정하지 않음.
- `lifecycle_unit_v1.log`, `analysis_unit_v1.log`, `continuity_unit_v1.log`: ROS 없는 Windows 모의 단위 검사 각각 6·5·3개 통과. 실제 executor/네트워크 실행 증거가 아님.

초기 분석 검사에서는 Windows Temp 파일 fixture 권한으로 4개 오류가 났다. 파일 쓰기가 필요 없는 메모리 fixture로 바꾸고 다시 검사했다. 이를 센서/ROS 오류로 해석하지 않는다.

## 실행·회수 근거

- `bundle_manifest_v1.json`: 센서 bag 제외 소스 69개. 원격 기존 소스는 별도 보존했고 전송 파일 해시를 대조했다. 묶음 SHA-256 `a6d7b6b55af80494adaf9f9f26dc863d9ebc580f24d836bb684f4a4626e41c2d`.
- `jetson_execution_v1.log`: Humble 2개 패키지 재빌드, 회귀 129개 통과/2개 제외, legacy A→retained A→retained B→legacy B 및 수신 감사. 로그 SHA-256 `c98eaab58cfd01f61756bb3c37fcea487434916d037046744bdc27d95439c29a`.
- `jetson_results/`: 4회 결과·소스/매개변수 스냅샷·엔코더 발행 시각·콜백 계측·자원 로그와 원격 비교 집계. 회수 tar.gz는 Git 제외 `build/`에 보존, 원격/로컬 SHA-256 `e07520d5bccbf89c2f6d0957a170ca2aa2611b6fa91a0e4007849e3d7fc84ca1` 일치. 원시 센서 bag 미포함.
- `pc_regression_v1.log`, `pc_retained_v1/`: PC 회귀 113개·retained 1배속. 기록 전달 감사 첫 잘못된 CLI 인자는 사용법 오류로 끝났고 올바른 positional 인자로 완료했다.
- `delivery_analysis_v2.json`: Jetson 4회와 PC 1회의 발행→콜백 및 최장 지연 동안의 다른 콜백 실행을 대조. `performance_v1.json`은 Jetson 주행 구간 명령/계산 시간 집계다. 기존 자료를 덮어쓰지 않았다.
- `pc_delivery_v1.json`, `pc_performance_v1.json`: PC 단독 집계.
- `jetson_transport_environment_v1.log`, `jetson_transport_environment_v1_dpkg_plain.log`: 실제 RMW 식별과 패키지 버전. 최초 dpkg 출력 포맷 오류의 빈 버전 로그는 정상 버전 조회 결과로 사용하지 않는다.
- `analysis_unit_v2.log`: 후처리 집계에 추가한 지연 중 콜백 경계 검사 포함 6개 통과. 이 분석 스크립트만 실행 묶음 이후 확장했으며 제어기/재생기 실행 소스는 변경하지 않았다.

기본/변경 방식 모두 한 회씩 주행 중 보호 정지가 재현됐다. 발행 추적과 `executor_mode=retained`는 비교 옵션이며 채택한 해결책이 아니다. 상세 판정·표·한계는 [A3 보고서](../../../../docs/simulation/JETSON_REPLAY_2026_09_28.md#엔코더-발행-추적과-executor-교차-비교--929-재개-결과)를 따른다.
