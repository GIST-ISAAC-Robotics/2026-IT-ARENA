# 선택형 최근 관측 보완: 저장 입력 재생

새벽 `g1_corridor_solo_v1/run/follow_decisions.jsonl`의 2,837회 판단을 대조했다. 원본 입력 SHA-256은 `9a44e3b8eb38f4358652c0b416eef6b947901a9086b0bc2ef532bf10f226a57d`다.

- `baseline.json`: 새 옵션 OFF, 2,837/2,837 판단 동일. 원래 실패 판정을 바꾸지 않는다.
- `enabled/report.json`: 새 옵션 ON의 반사실 대조. 341회 명령 tuple 차이, 관측 보완 선택 3회. 첫 seq 2996(59.92 s)은 정지에서 0.2 m/s·조향 0.19123 rad로 바뀌었다.
- 뒤의 입력은 **원래 정지 명령을 받은 차량의 측정 기록**이다. 선택 후에도 이 입력을 그대로 재생했으므로 새 폐루프 주행·코너 통과·완주를 의미하지 않는다. 341회 모두 전진한 것도 아니다.
- 최대 판단 비용 77.32 ms는 PC 오프라인 관측값이며 20 ms 주기 충족이나 Jetson 성능이 아니다. 이송·ROS·독립 보호층 비용까지 포함한 수치도 아니다.

실행은 `run_baseline.sh`, `run_enabled.sh`에 보존했다. 전용 bash 파일에서 ROS overlay를 source한 뒤 기존 `PYTHONPATH`를 유지하며 checkout 경로를 앞에 붙인다. 상위 PowerShell에서 `$PYTHONPATH`를 먼저 확장하면 ROS 메시지 import가 사라질 수 있다. 별도 v1/v2 수집 실패는 보존했다.

`pre_replay_manifest.json`과 `pre_replay_source_snapshot.zip`은 실행 전 입력/소스를 보존한다. 실행 후 대조에서 변경 0, enabled 도구도 입력·자신의 사용 소스 불변을 확인했다. 주행기 기본값은 false이고 원시 unknown/마스크·센서 스펙·물성을 바꾸지 않았다. 원시 trace는 기존 Git 제외 정책을 따른다.

전체 설계·한계·후속은 [관측 충분성 보고서](../../../../docs/autonomy/G1_OBSERVATION_ENVELOPE_2026_10_16.md)를 따른다.
