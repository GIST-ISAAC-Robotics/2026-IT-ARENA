# ESP32-S3 대상 컴파일·심볼 검사

2026-10-22, 실제 보드 없이 수행했다. [공식 ESP-IDF v6.1](https://github.com/espressif/esp-idf/tree/fff9895c82d744c7237be8847347bdd1b07c6643)의 [tools.json](https://github.com/espressif/esp-idf/blob/fff9895c82d744c7237be8847347bdd1b07c6643/tools/tools.json)에 지정된 `esp-15.2.0_20251204` GCC 15.2.0을 별도 WSL 캐시에 준비했다. [다운로드 출처·크기·SHA-256](toolchain.json)을 대조했으며 시스템 패키지·전역 PATH·ROS는 변경하지 않았다. 최종 MCU 프레임워크의 선택/확정은 아니다.

## 결과

- [도구 준비 종료](setup_exit.txt) 0, [컴파일 종료](probe_exit.txt) 0, [전후 소스](source_hash_diff.txt) 불변.
- [실행 명령](run_probe.sh): C++17·`-Os -fno-exceptions -fno-rtti -fstack-usage`, 경고 오류 처리. 합성 코어 API를 실제로 인스턴스화한 `actuation_target_probe.cpp`와 일반 빌드의 시험 주입 API 비노출을 컴파일했다.
- [ELF](target_elf_header.txt)는 **ELF32 / little-endian / Tensilica Xtensa / relocatable**이다. 완성 펌웨어 ELF가 아니다.
- 대상 오브젝트에서 필요한 `std::from_chars(..., double&, ...)` [미해결 심볼](target_undefined.txt)과 공식 대상 libstdc++의 [정의](from_chars_definition.txt)를 모두 확인했다. 실제 펌웨어 링크와 MCU에서의 정확 반올림/실행시간/내부 할당 검증은 아직 별개다.
- host 일반 빌드 API 실행도 종료 0이다. 대상 오브젝트는 실행하지 않았다.

## 대상 ABI 크기·한계

[대상 심볼](target_symbols.txt)의 크기 배열에서 읽은 값이다. 배열은 크기 추출용이지 실제 동작 객체가 아니다.

| 타입 | 대상 크기(B) |
|---|---:|
| FrameDecoder | 1816 |
| FrameLink | 1888 |
| CommandParser | 3656 |
| Command | 1144 |
| ActuationReceiverCore | 1616 |
| ReceiverConfig | 112 |

`arena_probe_construct`의 [함수별 스택 보고](probe_target.su)는 1776 B, `receive` 96 B, parser 주요 함수 80 B다. 호출 사슬·RTOS·라이브러리·인터럽트까지 합친 task 최악 스택이나 실제 실행시간은 아니다. 이 때문에 검사 앱의 큰 parser scratch는 전역 저장 공간으로 분리했다.

[오브젝트 크기](target_size.txt) text 10255 B / data 0 / bss 10232 B에는 위 크기 추출용 배열이 포함되므로 **실제 MCU 펌웨어 메모리 사용량으로 제시하지 않는다**. 대상은 binary64 소프트웨어 연산 보조 함수도 참조한다. 연산 비용은 보드에서 별도 측정해야 한다.

도구 설치 경로는 `/home/jinhyeong/.cache/it-arena/xtensa-esp-15.2.0_20251204`이며, [준비 스크립트](prepare_toolchain.sh)로 출처를 추적한다. 도구 자체는 저장소에 넣지 않는다. 다음은 출력 드라이버가 없는 검사 전용 SDK 앱의 링크 확인이다.
