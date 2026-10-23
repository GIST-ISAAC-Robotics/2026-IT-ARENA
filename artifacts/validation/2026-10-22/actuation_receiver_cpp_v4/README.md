# 수신 코어 API 수정본 — 잔여 검증 완료

2026-10-22 PC/WSL 검증이다. [v2](../actuation_receiver_cpp_v2/README.md)의 코어·API 수정본을 유지하고, 변이 주입 위치 검사기와 [v3](../actuation_receiver_cpp_v3/README.md)의 잘못된 시험 문자열을 고친 뒤 미완료 검사를 수행했다. 이전 실패를 통과로 바꾸지 않는다.

## 결과

| 검사 | 실제 결과 | 근거 |
|---|---|---|
| 변이 주입 위치 사전 회귀 | 15 통과 | [mutation_anchors.log](mutation_anchors.log) |
| 의도적 결함 검출 | 13/13 검출 | [mutants.json](mutants/mutants.json) |
| `-O2` 직접 API·일반 빌드 검사 | 10 통과 | [api_o2_tests.log](api_o2_tests.log) |
| 전체 회귀 | **3,635 통과**, 113.85초 | [full_tests.log](full_tests.log) |
| 실제 묶음 종료 | **0** | [runner_status.txt](runner_status.txt) |
| 실행 전후 소스 | 차이 없음 | [source_hash_diff.txt](source_hash_diff.txt) |

정상 코어의 sanitizer 오류는 발견되지 않았다. 변이체 중 `count_delta_wraps_int64`에서만 예상한 UBSan 오류가 발생했다. 이는 일부러 넣은 정수 넘침의 검출이며 정상 코어 실패가 아니다. 변이 검사기는 주입 위치가 없거나 둘 이상이면 실패하므로 소스가 바뀐 뒤 결함을 넣지 못한 경우를 검출 성공으로 세지 않는다.

## 재사용과 재현 범위

- [실행 스크립트](run_checks.sh)는 v2 바이너리 8개와 입력 헤더/driver·README 11개의 SHA-256이 같은지 먼저 검사했다. [바이너리 검사](reused_binaries_check.log), [입력 소스 검사](reused_sources_check.log), [대상 해시](reused_sources.sha256).
- 정상 코어의 ASan/UBSan·O2 66사례·8,680이벤트·receive 3,667건 불일치 0은 **v2에서 수행한 대조**다. 코어가 바뀌지 않았으므로 같은 export를 다시 실행하지 않았다. v4 전체 회귀는 정상 코어/API 검사를 포함한다.
- 수신 헤더 SHA-256: `69e7c569a1ba6a953c4d76614dbb653cf40f96b3dd705c108d33f9f2c6020b17`.
- 당시 소스는 `source_snapshot_before.tar.gz`, 전후 목록은 `sources_before.sha256`·`sources_after.sha256`에 보존했다. 본 README와 후속 현황/인수인계 문서 정리는 시험 이후에 작성했다.
- 수정 전 직접 API 실패는 [별도 기준선](../actuation_receiver_api_v1/README.md)에 보존했다. v1·v2·v3의 회귀 개수를 이번 결과에 합산하지 않는다.

## 한계

PC 의미 대조·API 범위 검증이다. 실제 ESP32 크로스 컴파일, UART/USB, PCNT·타이머·ISR 동시성, 최악 실행시간, MD20A/서보 출력, 전기 STOP과 차체 정지는 미검증이다. 운영 Python·ROS·Gazebo 주행은 변경하지 않았다. 다음 단계는 [구동 계약 §15의 실물 이관 준비](../../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#155-실물-연결-전-인수인계-순서)를 따른다.
