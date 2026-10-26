# 구매 부품 CAD 수집 — 2026-10-24

사용자 제공 구매 목록의 실물 부품 8종을 GPT가 직접 조사했다. 배송비 2행을 제외했다. 원본 Excel SHA-256은 `dade4fa3e8654efdc687fb737815eaa26403d566d6dd83e4cee065f774069483`이며 수집 전후 동일하다. 기존 구매안·제어 설정·차량 모델은 변경하지 않았다.

| 부품 | 확보 결과 | 근거 |
|---|---|---|
| RPLIDAR C1 | 공식 STP·데이터시트 | [SLAMTEC 지원](https://www.slamtec.com/en/support), [공식 모델](https://download-en.slamtec.com/api/download/rplidar-c1-model-3d-stp/1?lang=netural) |
| Arducam B0183 | 판매처 제공 STEP | [B0183 모델](https://www.uctronics.com/download/Mechanical_Drawing/B0183.STEP), [제품 연결 근거](https://www.robotics-3d.com/arducam/9169-imx219-low-distortion-m12-for-rasp-and-jet.html) |
| Adafruit 4438 | 커뮤니티 보드 STEP 확보, 공식 Eagle BRD·SCH·치수도 보존 | [Hawkry Zhang의 rough 보드 모델](https://grabcad.com/library/adafruit-lsm6dsox-ism330dhc-lsm6dso32-1), [공식 PCB](https://github.com/adafruit/Adafruit-LSM6DSOX-PCB) |
| VND018 / YD-ESP32-S3 | 커뮤니티 STEP·제조사 치수 PDF·Altium PCB | [커뮤니티 모델과 제작자](https://github.com/shkuznetsov/YD-ESP32-S3), [제조사](https://github.com/vcc-gnd/YD-ESP32-S3) |
| Bus Servo Adapter (A) | 공식 STEP | [Waveshare Resources](https://docs.waveshare.com/Bus_Servo_Adapter_A/Resources-And-Documents) |
| MD20A | 공식 STEP·IGES·데이터시트 | [Cytron Resources](https://sg.cytron.io/p-20amp-6v-30v-dc-motor-driver) |
| MB4266-24180 뒤쪽 샤프트형 | 판매처 치수도, 공개 CAD 미확보 | [구매 제품](https://www.motorbank.kr/goods/goods_view.php?goodsNo=1000034088) |
| PGE-213 | 판매처 치수도, 공개 CAD 미확보 | [구매 제품](https://www.motorbank.kr/goods/goods_view.php?goodsNo=1000009561) |

총 5종의 3D CAD와 나머지 3종의 보완 자료를 사용자 Downloads의 `IT-ARENA_CAD_2026-10-24.zip`에 수집했다. 24파일·25,393,426바이트이며, 자세한 사용 안내·출처는 묶음의 `README.md`, 다운로드 이력은 `download_manifest.json`, 파일별 해시와 형식 검사 결과는 `validation.json`에 있다. 외부 CAD·원본 Excel을 저장소에 추가하지 않았다.

IMU 비공식 모델 재조사 결과를 추가한 전달본은 `IT-ARENA_CAD_2026-10-24_v2.zip`이다. 28파일·25,406,668바이트이며 최초 ZIP은 보존했다. 추가 Xirkit STEP/STL은 **25.6 × 17.8 × 4.6 mm 직육면체**다. 파일 검사에서 STEP 면 6개, STL 삼각형 12개·고유 꼭짓점 8개를 확인했다. 장착 구멍·커넥터가 없어 대략적인 공간 배치용으로만 사용하며 완성 부품 CAD 5종에 합산하지 않는다. v2 ZIP CRC·압축 내용 해시 일치·원본 Excel 불변을 재확인했다.

[GrabCAD 보드 모델 원본 링크](https://grabcad.com/library/adafruit-lsm6dsox-ism330dhc-lsm6dso32-1)는 [같은 계열용 케이스 설명의 색인](https://www.stlfinder.com/model/case-for-adafruit-lsm6dsox-ism330dhc-lsm6dso32/8011284?q=lsm6dsox)에서 찾았다. 초기에는 웹 조회와 브라우저 모두 CloudFront 403 응답이었지만, 사용자 요청으로 재시도한 뒤 정상 접속하여 원본 ZIP과 `Adafruit_ISM330DHXC.STEP`을 확보했다. 원본 페이지에서 제작자 Hawkry Zhang·게시일 2021-04-18·rough 3D model 설명을 직접 확인했다. 동봉 미리보기에는 기판·장착 구멍 2개·양쪽 커넥터·주요 칩이 표현되며 STEP 검사에서는 solid BREP 34개·면 700개를 확인했다.

설명은 ISM330DHCX 4502를 연결하며, [Adafruit 공식 안내](https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu/downloads)는 LSM6DSOX와 ISM330DHCX의 fabrication print가 동일하다고 명시한다. 이는 기판 형상 후보로 검토할 근거이며 구매품의 모든 부품·높이 일치까지 보장하지 않는다. STEP 정점 좌표 범위는 약 25.40 × 17.78 × 4.735 mm이며 CAD 커널의 곡면 포함 경계나 실측값은 아니다. 로컬 `03_Adafruit_4438/grabcad_Hawkry_Zhang/README.md`에 적용 범위·검사 결과를 기록했다.

**최신 전달본은 `IT-ARENA_CAD_2026-10-24_v3.zip`(33파일·27,211,573바이트)이며 부품 CAD 6종을 포함한다.** 사용자 요청에 따라 Xirkit 후보 폴더와 자료를 현재 전달본에서 제외하고 안내문·검증 결과를 갱신했다. 원본 ZIP들과 v2는 보존했다. IMU·ESP32는 비공식 커뮤니티 자료, B0183는 판매처 제공 자료다. v3의 ZIP CRC·파일 해시 일치·제외 자료 부재·원본 Excel 불변을 확인했다. IMU STEP SHA-256은 `40b27418b293e2515d8027f95580610e094b872dd8c64adc2b60209f1766fb71`이다. Fusion 실제 가져오기와 실물 판본·고정 구멍별 치수·공차 검증은 수행하지 않았다.

로컬 정리 상태: 제거 대상은 묶음 내부 `_research/removed_4438_candidates`로 이동되어 전달 ZIP에 포함되지 않는다. 사용자의 명시적 영구 삭제 요청도 있었으나 실행 도구의 정책 차단으로 해당 내부 폴더의 영구 삭제는 완료하지 못했다.

검증은 STEP 시작/종료 표식·기하 엔티티, 배포 및 최종 ZIP CRC·압축 내용과 로컬 파일 해시 일치, Eagle XML·PDF 구조·이미지 형식까지다. Fusion 실제 가져오기·기하 유효성 전체 검사·실물 조립 검증은 하지 않았다. B0183 STEP의 15핀 FPC 판본, ESP32 커뮤니티 모델의 헤더·보드 판본, 모터의 뒤쪽 샤프트 옵션은 실제 납품품과 대조해야 한다. PGE-213 케이스의 축방향 치수는 남아 있다. 부품 CAD 수집은 기존 하드웨어 검증을 대체하지 않는다.
