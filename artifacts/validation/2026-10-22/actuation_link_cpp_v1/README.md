# FrameLink 후보 1차 대조 — 부분 프레임 절대 기한·연결/오류 정책

- [명령](run_checks.sh): WSL Ubuntu, [g++ 13.3.0](compiler_version.txt), C++17 `-Wall -Wextra -Werror -pedantic`. 링크·기존 프레임 driver는 ASan/UBSan(`-O1`), 링크 driver는 별도 `-O2` 비교 빌드. [컴파일 로그](compiler.log) 비어 있음, [binary SHA-256](binaries.sha256). 로컬 실행 파일만 Git에서 제외한다.
- [실제 종료](runner_status.txt) 0, [소스 해시 차이](source_hash_diff.txt) 없음. 실행 전 `src/scripts/tests/config` 전체 [해시](sources_before.sha256)·[스냅샷](source_snapshot_before.tar.gz)을 보존했다.
- [관련 회귀](tests.log) **198 통과**(새 링크 14 + 기존 프레임 C++·전송·포팅 벡터·계약·ROS 노드·링크 예산), [전체 회귀](full_tests.log) **3,593 통과**. 두 C++ binary 환경변수를 지정해 C++ 시험이 skip되지 않았다(로그 내 skip 0).
- [링크 대조](export/report.json): 링크 단위 **509사례**(수동 경계 사례 28·기존 프레임 사례 241·무작위 240), 참조 정책 **10사례**의 링크 연산을 포함해 driver **13,764줄·13,245연산**, C++/독립 oracle 차이 **0**. 수동 링크 기대값 65개, 정책 1,054이벤트·상태 checkpoint 43·링크 checkpoint 22를 **C++ 링크 출력으로 재생**해 차이 0. stderr 0 B. [`-O2` 빌드](export_o2/report.json)도 같은 결과.
- [기존 프레임 코어 회귀](wire_export/report.json): `reject_head` 분리 뒤에도 243사례·2,294연산 차이 0, Python JSON 경계 2,039회 차이 0.
- [변이체 검출](mutants/mutants.json): 기한 off-by-one, 조각마다 기한 연장, drain 없는 폐기, 재연결 시 부분 바이트 유지, 시계 역행 무시, 입력 예산 미고정, 단절 미계수, poll 미만료, 고장 해제 시 자동 연결의 **9/9 검출**. 원본 헤더 해시 불변. 변이 헤더·컴파일 로그를 보존하고 binary는 삭제했다.

## 판정 범위

- 시험 timeout(링크 단위 0·1·2·7·100·1000·`UINT64_MAX` tick, 정책 30 ms)은 **시험값**이다. 운영 timeout은 정하지 않았다.
- C++ `FrameLink`는 구동 허가를 다루지 않는다. 세션 경계 → 로컬 STOP 고정 → CLEAR → 별도 ARM은 Python 참조 정책이 기존 `ActuationReceiver`를 수정하지 않고 적용했다. 엄격 JSON·명령 검증도 기존 Python을 뒤에 둔 대조다.
- ROS 기본 경로·Python 런타임·실제 MCU/UART/USB·ESP32 크로스 컴파일·최악 실행시간·전기 STOP은 미연결·미검증이다. `firmware_verified=false`, `ready_to_drive=false`.

상세 해석은 [구동 계약 §13](../../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#13-1022-후속--부분-프레임-절대-기한과-연결오류-정책-후보)에 둔다.
