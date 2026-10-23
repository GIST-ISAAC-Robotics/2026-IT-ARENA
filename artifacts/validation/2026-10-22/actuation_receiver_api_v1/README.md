# C++ 수신 코어 API 경계의 수정 전 재현

기존 v1의 이벤트 동등성 통과와 별도로, 공개 코어 API를 직접 호출했을 때의 경계를 검사했다. [원본 헤더/시험 소스](baseline/source), 개별 stdout/stderr·종료 코드·해시를 `baseline/`에 보존했다. 이 실행은 **수정 전 실패 증거**이며 뒤의 성공으로 덮어쓰지 않는다.

- [종료 코드](baseline/exitcodes.tsv): 8개 중 6개 exit 1, 2개 exit 0.
- `invalid_feedback_mode`, `invalid_config_constructor`, `empty_boot_constructor`, `long_boot_constructor`: 잘못된 구성/boot 입력을 생성 시 거절하지 못했다.
- `null_boot_constructor`: [ASan/UBSan](baseline/null_boot_constructor.stderr.txt)이 null 역참조·SEGV를 보고했다.
- `clear_at_counter_exhaustion`: 실제 상태는 세대 소진 고장인데 CLEAR 응답이 수락으로 나왔다.
- `freshness_invalid_time`: exit 0이어도 [UBSan](baseline/freshness_invalid_time.stderr.txt)에 `INT64_MIN - 220000`의 signed overflow가 있었다. 정상 통과가 아니다.
- `valid_motor_arm_drive_stop`: 정상 모터축 ARM→DRIVE→STOP 대조는 exit 0·stderr 없음.

후속에는 위 경계를 수정하고 `invalid_boot_codepoint`, 일반 빌드의 시험용 상태 주입 API 비노출 검사도 추가했다. 후속 시험은 9개 직접 실행 사례+일반 빌드 1개로, 수정 전 8개와 동일 집합이라고 부르지 않는다. [수정본 v2](../actuation_receiver_cpp_v2/README.md), [최종 검증 경로](../actuation_receiver_cpp_v4/README.md).
