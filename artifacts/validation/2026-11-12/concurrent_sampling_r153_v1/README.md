# 동시 수신 C1 각도·점 수를 반영한 선택형 시뮬 입력

2026-11-12 [r146 실물 동시 수신](../sensor_concurrency_r146_v1/README.md)의 원시 CSV를 해시 대조하고 각도 배열만 추출했다. 거리·품질·실내 형상·센서 원시 시각은 프로필에 넣지 않았다. 기존 프로필과 원본은 그대로 두며 기본값을 변경하지 않았다.

752개 측정 회전의 점 수는492·496·500·504·508·512·516·520·524였다. 각 점 수의 첫 사례와 시간축에 고르게 고른32개를 합쳐 중복 없는40배열을 선택했다. 이 선택은 드물게 나타난 점 수를 확인하기 위한 형식/기하 시험용이며 원래 출현 빈도를 재현하는 확률 모델이 아니다. [선택 근거](selection.json)·[프로필](profile.json).

기존 `NativeRevolutionAssembler`와 native 직렬화/복원에2,001개의 합성 고밀도 스캔을 넣어40회전을 만들었다. 모든 배열에서 점 수·CW 순서·거리/실제 source 각도 짝·과거 source 시각을 유지했고 버린 회전은0개였다. [결과](validation.json). 센서 노이즈·실제 교통 주행 검증은 아니다.

`rotation_rate_hz=10`, `time_model=period_divided_by_count`, `hardware_timing_verified=false`를 유지한다. 실제 C1 SDK 첫시각+200μs의 배치 간 겹침 문제나 취득 시각 교정이 해결됐다는 뜻이 아니다. 런타임은5000칸 source 광선 중 실제 선택한 방향을 거리와 함께 보내며 목표 각도로 바꿔 붙이지 않는다. 후속 센서 변화 대표에서 `--lidar-native-sampling-profile artifacts/validation/2026-11-12/concurrent_sampling_r153_v1/profile.json`으로 명시 선택할 수 있다.
