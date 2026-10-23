# ESP32-S3 검사 앱 빌드 v3

실제 ESP32-S3용 ELF/BIN **빌드 성공**, 장치 업로드·실행은 하지 않았다. 구동 펌웨어가 아니며 `firmware_verified=false`, `ready_to_drive=false`다.

- SDK: Espressif ESP-IDF v6.1, commit `fff9895c82d744c7237be8847347bdd1b07c6643`.
- 컴파일러: 공식 Xtensa `esp-15.2.0_20251204`, C++17, 예외/RTTI 비활성. [대상 객체 검사·다운로드 해시](../actuation_xtensa_probe_v1/README.md).
- 격리 환경: WSL 사용자 캐시 `~/.cache/it-arena/idf-6.1-smoke`, 전용 `python-v3`. 시스템 Python/ROS/Jetson은 변경하지 않았다.
- 준비: `prepare_sdk.sh` → `sdk_setup_exit.txt=0`, 전용 환경 `pip check` 성공.
- 빌드: `scripts/build_actuation_smoke.sh` → `build_exit.txt=0`. `build_source_snapshot.tar.gz`, before/after SHA-256와 빈 `build_source_hash_diff.txt`로 당시 소스를 보존했다.
- `elf_header.txt`: ELF32 little-endian, EXEC, Tensilica Xtensa. 링크된 `std::from_chars(double)` 심볼 확인. `undefined_symbols.txt`의 3개 이름은 SDK CMake에서 `-u`로 명시한 이름이며 ELF에 relocation은 없다. 이 목록이 비어 있다고 주장하지 않는다.
- `idf_size.txt`: BIN 0x2a560(173,408) B, IDF reported image 173,287 B, DIRAM 46,447 B. SDK·검사 코드 전체의 정적 배치이며 차량 펌웨어/실행 힙·스택 사용량이 아니다. 기본 2 MB flash 구성은 **검사 빌드값**으로 납품 N16R8의 전체 메모리 설정을 확정한 것이 아니다.
- 공유 합성 사례 20개는 호스트 ASan/UBSan에서 통과했다([v2 host 로그](../actuation_idf_smoke_v2/host.log)). pytest의 sanitizer/size-optimized 2개도 통과했다(`pytest_smoke.log`). **대상 보드에서 20개가 실행됐다는 뜻이 아니다.**

## 실패를 보존한 과정

1. [v1](../actuation_idf_smoke_v1/): 전용 Python 경로가 없을 때 SDK 환경 검사에서 실패. 소스 clone은 성공했다.
2. [v2](../actuation_idf_smoke_v2/): WSL Python에 ensurepip가 없어 기본 venv 구성 실패. 기존 설치를 지우거나 시스템 패키지를 바꾸지 않았다.
3. v3: 새 전용 venv를 `--without-pip`로 만들고 [공식 get-pip](https://pip.pypa.io/en/stable/installation/)으로 그 환경에만 pip를 설치한 뒤 SDK 의존성을 준비했다. 실패 폴더는 보존했다.

원시 설치 로그와 ELF/BIN은 이 폴더의 Git 제외 규칙으로 로컬 보존한다. 숫자 변환의 대상 실행 결과·내부 할당·최악 실행시간·실제 stack 여유·UART·PCNT·PWM/DIR·서보·전기 STOP은 미검증이다. SDK 기본 로그 출력 외 장치 제어를 구현하지 않았으며 자동 flash 명령은 제공하지 않는다.
