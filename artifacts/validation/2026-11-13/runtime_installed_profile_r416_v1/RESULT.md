# r416 실장 프로필 빌드 연결 결과

2026-11-13. **관련18검사·합성 프로필 ESP-IDF 빌드1회를 통과했고 검증된6파일을 공통 코드에 이관했다.** 실제 영점/모터 방향/PI를 생성하거나 장치에 업로드하지 않았다.

## 구현과 확인

`runtime_installed_profile.py prepare`는 기존 장착 서보 JSON·별도 명시 board JSON·같은 assembly ID를 받아 원본2개와 C++ header2개, source/내용 해시를 보존한다. 기존 영점 export와 C++ `RuntimeBoardProfile.valid()`를 재사용한다. 새 PI/기구/동역학 판정선이나 임의 기본값은 만들지 않았다.

`build_runtime_drive.sh`의 선택 인자 `--installed-profile DIR`로만 주입된다. 미지정은 기존 빈/미교정 구성이다. header가 우연히 존재해도 활성화하지 않으며 preview 앱에서는 옵션을 거절한다. 선택된 fresh 빌드 사본에서 프로필/소스 해시와 기존 C++ valid를 확인한 뒤 IDF로 넘긴다. protocol/main/STOP/부팅 처리는 바꾸지 않았다.

- 관련18검사 통과: 기존 영점 export2개와 신규16개. 기본 비활성·명시 값/영점의 C++ 전달·assembly 불일치·NaN/키/변조·덮어쓰기 거절 등을 확인했다.
- 합성 `synthetic-not-installed` 프로필로 ESP-IDF 빌드1회 exit0, 약100.48초. [실행 보고서](verify_build_v1/extracted/report.json)의 `sources_unchanged=true`, `hardware_executed=false`, `firmware_flashed=false`를 유지한다.
- [실제 compile 선택](verify_build_v1/compile_selection.json)에서 main.cpp의 `-DARENA_RUNTIME_INSTALLED_PROFILE=1`, 해당 include 경로와 CMake cache ON을 확인했다. main 원문·복사된 두 header 해시가 manifest와 같다. 이 확인은 기존 산출물 읽기이며 빌드를 다시 하지 않았다.
- [공통 이관 영수증](integration.json)의6파일과 검증에 쓰인 의존 소스가 바이트 동일하다. 공통 CLI `--help`도 정상 실행했다. 동일 IDF 빌드를 공통 경로에서 반복하지 않았다.

실행 담당의 첫 shell 명령은 인용 오류로 실제 검사 전에 종료됐다. 새 LF wrapper에서 실제18검사/IDF는 각각1회만 실행했다. 결과 ZIP과 로그/소스 snapshot을 보존한다.

## 실물에 남는 부분

빌드 연결은 준비됐지만 **실제 장착 영점·서보 count방향/한계·모터/엔코더 방향·PI/duty/속도 상한의 확인과 채택은 남는다.** 사람이 확인한 값의 JSON을 준비한 뒤 [사용법](README.md#입력-계약과-사용법)으로 생성·빌드한다. 엔코더 부호는 firmware와 host에서 각각 반전해 서로 상쇄하거나 중복 적용하지 않도록 원천 의미를 맞춘다.

합성 프로필 바이너리는 설치용이 아니다. 검증된 프로필을 넣은 바이너리는 기존 ARM/DRIVE 조건에서 실제 출력을 할 수 있으므로, 빌드 성공을 flash/출력 허가·장착 안전·물리 STOP 확인으로 확대하지 않는다. 이번에는 어떤 장치에도 송신하지 않았다.
