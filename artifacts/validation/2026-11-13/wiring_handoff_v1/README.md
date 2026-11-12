# ESP32 조립 배선 인계 정적 검증 — 2026-11-13
> 이 검증은 최초 세 장 구성 당시의 기록이다. 이후 사용자 요청으로 [두 장 구성](../wiring_handoff_two_sheet_v1/README.md)으로 통합했다. 당시 생성물은 비공개 `build/wiring_handoff_20261113/three_sheet_before_merge/`에 보존했고 아래 해시는 당시 파일을 가리킨다.


[배선도](../../../../docs/hardware/schematics/esp32_gpio_wiring_v1.html) · [조립 설명](../../../../docs/hardware/ESP32_GPIO_WIRING_2026_11_11.md) · [검증·파일 해시](validation.json)

- 11 GPIO 중복/제외/예약 충돌 없음. 3V3 두 핀·GND 네 핀과 T3V3/T5V/TGND 접속 대상 검사.
- J01/M01 제조사 Preliminary PDF p.3 표·그림에서 CN1 입력, CH1~6↔CN2~7 및 정격 확인. 원본 해시는 JSON에 보존하며 PDF 원본은 gitignored build에 저장.
- 서보 전원 M01 CH5(CN6), 어댑터↔서보 3선 전용 케이블 명시. CH6(CN7)은 예비.
- SVG 3장을 1700 px PNG로 렌더링해 육안 검토. HTML 안 SVG 텍스트 221개에 대해 경계 이탈/상호 겹침 0을 브라우저에서 확인. 로컬 연결 파일·Markdown 앵커 검사 통과.
- 문서·도면 작업이다. 장치 접속/구동/펌웨어 변경은 없고, 새 헤더 분배·최종 하네스의 통전/부하 검증을 대신하지 않는다. 기존 벤치 결과는 해당 11/12 보고서로 구분한다.
