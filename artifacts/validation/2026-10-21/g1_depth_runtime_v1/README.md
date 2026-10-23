# 깊이 창 함수 운영 반영 검증

`run_checks.sh` 1회: 전체 3,156 pytest, ON 3,001/OFF 2,837판단 일치, 정상 종료 0, `source_hash_diff.txt` 비어 있음. 양쪽 프로필의 소스 snapshot을 보존했다.

- ON total p50/p95/max: 5.285791/8.630012/22.604435 ms, >20 ms 5건.
- OFF total p50/p95/max: 5.158265/7.955992/16.661832 ms, >20 ms 0건.
- ON/OFF는 다른 원시 입력이며 각 모드 과거 기준과의 동등성을 검사했다. 원점 fail-closed 정정 **전** 소스다.
- 실시간 ROS/독립 보호/Gazebo/Jetson 마감 통과가 아니며 코너 완주 실패는 그대로다.

[기준 보고서](../../../../docs/autonomy/G1_OBSERVATION_CONTRACT_2026_10_21.md)
