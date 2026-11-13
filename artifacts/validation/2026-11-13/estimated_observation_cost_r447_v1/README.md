# r447 저장 입력 관측 보완 비용

최신 공통 r440/r444/r446의 `Evaluator('current', header)` 전체 호출을 실제 Jetson의 별도 native 빌드에서 비교합니다. 실제 센서 입력·주행·노드 전체 ROS 처리율 검증이 아닙니다. 입력은 r437 96개와 r426의 정상 양수·기존 corner·음수 측정 대표 3개이며, 저장 시뮬레이션 수치의 원계약과 명시 estimated 계약 사본 각 99개를 한 번씩 계산합니다.

- 원 입력: r438 `inputs/{r437,r426}_currents.jsonl` 및 두 header. x86 기대 출력: r440 `verify_v1/saved_result/rows.jsonl`의 동일 dataset/trace_seq.
- 변환: r444 `verify_saved.py`를 바이트 그대로 `contract_reference.py`로 보존합니다. `timing_verified=false`, `host_estimated_v1`, `unverified_estimate`와 명시 observation scope만 바꾸며 원시 scan·wheel/history·epoch·기하 수치를 바꾸지 않습니다. 저장 wheel 수치는 실측이 아닙니다.
- 타이머: `perf_counter_ns`로 `Evaluator.__call__` 진입 직전부터 반환 직후까지 측정합니다. restore/scene/current recheck/freeze가 포함됩니다. 입력 hash·출력 직렬화·비교·프로세스 시작·Evaluator 생성·IPC는 제외하고 생성 비용은 별도로 기록합니다.
- 워밍업 추가 호출은 0입니다. 계약별 독립 프로세스이며 첫 호출과 이후 호출, dataset별 첫 호출을 분리합니다. p95는 정렬 후 nearest-rank입니다. 측정 순서는 simulation 99회 → estimated 99회로 고정되므로 순서/온도/주파수 차이를 제거한 교차 실험이 아닙니다.
- 원 x86 비교 규칙과 전체 차이는 r411 `diff_utils.py` 그대로 보존합니다. 수치/region 차이가 나오면 원 실패를 보존하며 허가선을 바꾸지 않습니다. 같은 ARM의 최종 허가·이유·명령·경로도 별도로 대조합니다.
- 계측 종료 후에만 실제 ROS `LocalPursuit` estimated 거절, `G1G2Node` 및 독립 `LidarSafety` 생성자를 별도 프로세스에서 확인합니다. spin·센서 주입·host·양수 발행·장치 접근은 없습니다.

`prepared.json`과 `source_manifest.json`이 동결 입력 및 승인된 이관 SHA를 기록합니다. 원 실패/로그를 보존하고 완료된 비용 계산은 자동 반복하지 않습니다. Jetson 루트는 `/home/q/it-arena-r447-a`이며 장치·기존 동작·전력·센서·flash 설정은 변경하지 않습니다.
