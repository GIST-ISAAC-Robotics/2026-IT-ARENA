# LSM6DSOX 원문 계약의 PC 검사

2026-10-23. `source_manifest.json`의 고정 SHA 원문을 읽고 ST C 파일 전체를 fake-register callback으로 링크했다. 실제 장치·SPI·ROS·보드·Jetson은 사용하지 않았다. Adafruit 원문은 정적 검토만 했으며 실행하지 않았다.

- `run_checks.sh` 1회 exit 0, ASAN/UBSAN·컴파일 stderr 비어 있음, 소스 전후 해시 차이 0 B.
- ST raw signed 3축·7개 32비트 timestamp·버스 오류 반환/호출자 출력 불변·BDU write/readback을 확인했다. fake callback 결과가 실제 전기/전송의 정상 동작을 보장하지 않는다.
- 9개 FS별 변환 함수의 int16 전체 입력 **589,824개를 한 번의 산술 대조**로 검사했다. 이를 589,824개의 별도 시스템 회귀/주행 시험이라고 세지 않는다.
- ±500 dps raw 1,000 → 17,500 mdps → 0.3054326191 rad/s. ±4 g raw 1,000 → 122 mg → 1.1964113 m/s². FS 채택·센서 교정 결과가 아니다.
- timestamp getter의 uint32 전체 범위와 `from_lsb_to_nsec(int16_t)`는 표현 범위가 다르다. probe는 명시적인 `-1` 인수에서 −25,000 ns, 별도 uint64 곱셈에서 UINT32_MAX×25,000 = 107,374,182,375,000 ns를 확인했다. 32비트를 16비트로 축소하는 암묵 변환을 실행한 증거로 설명하지 않는다. 실제 wrap/reset·시계 매핑·FIFO 항목 시각은 미검증이다.

`upstream/`과 binary는 로컬 보존·Git 제외다. 원문 재회수 URL/해시는 manifest에 있다. ST 원문/링크 산출물을 재배포할 때에는 해당 판본의 [BSD 3-Clause LICENSE](https://github.com/STMicroelectronics/lsm6dsox-pid/blob/2e86793125f7f46b6195fce93a2b3cb76a51e564/LICENSE)를 함께 보존한다(로컬 `upstream/ST_LICENSE`). 이 폴더의 probe는 별도 fake bus/검사 코드이며 제조사 드라이버 대체품이 아니다.

입력 계약·Adafruit 함수 시각/읽기 반환·현재 소비자 제한은 [C2 §9](../../../../docs/hardware/HARDWARE_CAPTURE_AUDIT_2026_10_13.md#9-1023--lsm6dsox-입력을-연결하기-전)에 정리했다.
