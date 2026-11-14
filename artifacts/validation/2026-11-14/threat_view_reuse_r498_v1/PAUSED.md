# r498 중단 인계

사용자 즉시 종료 요청으로 중단했다. 새 계산/시험을 시작하지 않는다. **공통 미적용**이며 WSL/Jetson에는 접근하지 않았다. 아래 Windows 실행은 중지 메시지 도착 전에 모두 exit 0으로 완료했고 남아 있는 소유 실행 세션은 없다.

- 수정 stage: `traffic_solo_gap.py`, `local_follow.py`, `pass_release_trial.py`, 신규 `tests/test_threat_view_transfer.py`. `original/`에 수정 전 사본, `source_manifest.json`에 852개 원본 SHA를 보존했다. 공통 코드/상위 문서는 수정하지 않았다.
- 이번 eligible 계산 결과를 같은 `_forward_command`의 `select`에 명시 전달한다. `select` 안 ContextVar는 finally reset하며, 같은 scene 객체·동일 path 내용·동등 config에서만 소비한다. `_select` 상속/runtime wrapper 서명과 partial 상태 갱신은 유지한다.
- `focused_tests_v2.log`: 80 passed. 상속/runtime shadow·observer·불일치 fallback·중첩/예외 후 잔류 없음 포함. v1의 ROS 메시지 의존성 수집 오류를 보존했으며 ROS 검사를 수행한 것은 아니다.
- `windows_run_v3/{original,stage}`: 현재 공통 원본/후보 각각 원 r493의 508입력을 순서대로 재생 완료. generation 356 최초 reset 1회, 과거 상태 주입 없음. 입력 SHA `909c107fe700cf751e0a8c09b154bec09bff99140cd78b6b449014f8dfa829ac` 불변.
- **Windows 전용 검증**: 격리 NumPy 2.3.5. 원 header는 보존하고 파생 header의 `numeric_execution=python`, `current_corner_budget=off`만 사용했다(plan 검사). ARM/native 동일성·처리율·ROS/실물 증거가 아니다.
- `comparison.json`: 명령 exact **508/508**. 원 strict 전체 출력 **507/508**; 차이는 `normal.partial_observation.surface_correspondence.processing_ms` 1건. 기존 `comparable_decision`의 지정 처리시간 진단 제외 후 **508/508**. strict 상태 **437/508**; 차이는 `normal.partial_policy.surface_cache[2].processing_ms` 71건이다. 원 strict 결과를 성공으로 바꾸지 않았다. 상태 정규화 추가 집계는 미수행이다.
- 실제 view 호출 **545→508**, 재사용 **37 hit/0 fallback**. 느린 입력 seq1304/1311은 각각 **2→1**, scene/path/config 모두 일치. seq717은 원래 1회 유지. 시간값은 직렬화/계측 포함 Windows 실행기록이며 Jetson 성능 개선으로 사용하지 않는다.
- `windows_run_v1`, `windows_run_v2`는 상태 직렬화 도구의 method/cycle 오류로 첫 입력 뒤 종료한 실패를 보존했다. v3에서 callable 식별자와 순환 참조 경로만 기록하도록 harness를 고쳤다. 제어 결과를 주입하지 않았다.

재개 시 남은 최소 작업: stage diff/변경 SHA·`change.patch` 확정, 원본 최종 무결성 확인, root 검토. 비교 자체는 이미 완료됐으므로 508 재실행 불필요. Jetson 비용 비교는 별도 승인/자원 배정 전 미실행이다.
