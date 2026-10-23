# C++ 명령 표현 후보 1차 대조 — 엄격 JSON과 명령 필드

- [명령](run_checks.sh): WSL Ubuntu, [g++ 13.3.0](compiler_version.txt), C++17 `-Wall -Wextra -Werror -pedantic`. 명령·링크·프레임 driver는 ASan/UBSan(`-O1`), 명령 driver는 별도 `-O2` 빌드. [컴파일 로그](compiler.log) 비어 있음, [binary SHA-256](binaries.sha256). 실행 파일만 Git에서 제외한다.
- [실제 종료](runner_status.txt) 0, [소스 해시 차이](source_hash_diff.txt) 없음. 실행 전 전체 [해시](sources_before.sha256)·[스냅샷](source_snapshot_before.tar.gz) 보존.
- [관련 회귀](tests.log) **208 통과**(새 명령 10 + 링크·프레임 C++·전송·포팅 벡터·계약·ROS 노드·링크 예산), [전체 회귀](full_tests.log) **3,603 통과**, skip 0.
- [대조](export/report.json): payload 6,285(손으로 쓴 문법·경계 corpus, 기존 포팅 벡터 명령, 실제 `CommandProducer`·ROS `ActuationHost/VirtualMCU` 콜백 payload, 무작위 변이 6,000) + 상태별 receive 사례 3,667 + 기존 전송 계약 12 스트림(FrameLink→parser), driver **10,763줄 C++/독립 기준 차이 0**. 실제 운영 `Decoder`의 전달 여부·dict와 기준 판정 차이 0. 수동 기대값 209개 실패 0.
- **receive 동등성 3,667건 차이 0**: 원래 JSON을 운영 Decoder로 읽은 dict와, C++ 출력만으로 되살린 dict를 같은 `ActuationReceiver` 상태(ACTIVE/ARMED/DISARMED/ESTOP)에 넣어 결과·전체 status가 같다. 결과 분포: session 1,861·schema 561·invalid_message 390·invalid_drive 291·not_owner_or_armed 167·drive 126 등([report](export/report.json)의 `receive_outcomes`).
- [`-O2` 빌드](export_o2/report.json)도 같은 결과. 두 실행의 `stdin.txt`·`stdout.txt` SHA-256은 각각 동일하다(입력·출력 사본).
- 호스트 객체 크기: `CommandParser` 3,848 B, `Command` 1,144 B, `FrameLink` 1,888 B(x86-64).
- [변이체 검출](mutants/mutants.json) **17/17**: 중복 키 미검출·즉시 보고, surrogate 쌍 길이, 제어문자, UTF-8 surrogate, 상수 분류, bigint 포화, underflow→inf, age/ttl 조건, seq 경계, 선행 0, 잔여 데이터, 결과 미초기화(누출), 내부 키 미해제, **잘못된 DRIVE를 parser에서 폐기**(receive 동등성 67건 실패로 검출), bool을 v=1로 인정, 깊은 envelope command.

## 증거 실행 전 경위

예비 실행(임시 빌드, 증거 아님)에서 다음을 고친 뒤 위 실행을 했다.

1. `deep_objects` 사례가 1536 B를 넘어 기준·C++ 모두 `invalid_size`였다. 구현이 아니라 시험 fixture 오류이므로 250단으로 줄였다.
2. 첫 변이체 검사는 16/17이었다. surrogate 쌍을 길이에서 결합하지 않는 변이를 놓쳤는데, surrogate 사례가 모두 명령 필드가 아닌 키에만 있었기 때문이다. `client`/`boot`의 escape 쌍·raw astral·짝 없는 surrogate·64/65 코드포인트 경계 9사례를 추가했다. 이 변이는 1단계 길이와 저장이 어긋날 수 있음을 보여 주므로, 저장 루프에 64개 배열 경계 방어도 넣었다(정상 코드에서는 길이가 같음).
3. 작업 도구가 시험 소스의 일부 `\u` escape를 풀어 쓴 것을 발견해, surrogate escape 줄을 역슬래시를 명시적으로 조립하는 방식으로 다시 썼다. BMP/astral 문자 리터럴로 바뀐 줄은 Python 문자열 값이 같다.

## 판정 범위

- 상태기계(세대·임대·소유권·순번 판정 순서와 고장 고정), ack/offer 생성, ROS·MCU 연결은 이관하지 않았다. receive 동등성은 기존 Python 상태기계가 C++ 표현만으로 같은 판정을 내릴 수 있음을 보인 것이다.
- `from_chars`는 PC libstdc++ 13.3에서 Python과 비트 일치만 확인했다. 할당 여부는 allocator 계측을 하지 않았으므로 PC·ESP32 모두 미확인이다(코어 코드 자체에는 동적 할당이 없다). (10/22 검토 뒤 정정) 최악 실행시간은 측정하지 않았다. `firmware_verified=false`, `ready_to_drive=false`.

상세 해석은 [구동 계약 §14](../../../../docs/hardware/ACTUATION_CONTRACT_2026_09_22.md#14-1022-후속--엄격-json과-명령-필드-표현의-c-이관)에 둔다.
