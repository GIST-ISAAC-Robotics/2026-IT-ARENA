# 수신 코어 API 수정본 — 검사기 단계 실패 보존

구성/boot 입력, 공개 신선도 함수의 정수 범위, 세대 소진 CLEAR 응답을 수정하고 시험용 카운터 주입을 매크로로 분리했다. [실행 명령](run_checks.sh), [실제 종료](runner_status.txt)는 exit 2이며 **전체 묶음 통과가 아니다**.

- [관련 회귀](tests.log) 225 통과(직접 API 9개와 일반 빌드 1개 포함).
- [ASan/UBSan 대조](export/report.json)·[O2 대조](export_o2/report.json): 각각 66사례·8,680이벤트·receive 3,667건 불일치 0, 경계 7개 통과, 정상 코어 stderr 없음.
- [변이체](mutants/mutants.json)는 12/13 검출. `physical_press_ignored_when_latched`는 결함을 놓친 결과가 아니라, `pressed` 조건에 구성 검사가 추가되어 **옛 텍스트를 찾지 못한 검사기 실패**다(`pattern not unique`, 실제 일치 0곳).
- `count_delta_wraps_int64`의 UBSan 출력은 일부러 넣은 잘못된 뺄셈을 검출한 것으로 정상 코어의 sanitizer 오류와 구별한다.
- 스크립트는 변이체 단계에서 멈췄으므로 O2 API·전체 회귀는 미실행이다. [컴파일 로그](compiler.log)와 [소스 해시 차이](source_hash_diff.txt)는 비어 있다.

후속은 v2 core/API 바이너리와 그 입력 헤더/driver의 해시가 그대로임을 확인해 재사용한다. 과거 결과 파일은 수정하지 않는다. [v3 검사기 사례 오류](../actuation_receiver_cpp_v3/README.md), [후속 v4](../actuation_receiver_cpp_v4/README.md).
