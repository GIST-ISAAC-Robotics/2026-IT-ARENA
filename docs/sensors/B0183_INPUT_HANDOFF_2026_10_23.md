# B0183 실물 영상 입력: 형식·시각·첫 연결 순서

2026-10-23. 현재 저장소 소비자와 제조사 공개 API를 대조한 **이관 준비**다. 카메라/Jetson에 접속하거나 드라이버를 설치하지 않았다. RGB 입력·영상 작업자·출발/보호 한도는 변경하지 않았다.

## 1. 현재 코드와 연결해야 할 지점

현재 B0183 시뮬레이션은 1280×720·60 Hz·수평 75° 핀홀 가정이다. 실물 출력 모드별 크롭·화각·왜곡·노출·롤링셔터는 미측정이다. 구매 사양은 [기존 조사](../hardware/RGB_CAMERA_MODULE_RESEARCH_2026_09_18.md), 선택은 [ADR 0020](../decisions/0020-b0183-c1-procurement-baseline.md)를 유지한다.

| 구간 | 현재 요구사항 | 실물에서 확인할 것 |
|---|---|---|
| ROS 영상 입력 | `/camera/color/image_raw`, `rgb8` 또는 `bgr8`, 행 간격 `step`, 정확한 `height × step` 바이트 | Bayer/NV12/BGRx를 이름만 RGB로 바꾸지 않고 실제 색 변환. 행 패딩도 보존 |
| 인식 작업자 | [단일 작업·최신 대기 한 장](../decisions/0025-isolated-vision-processing.md), 최대 1920×1080×3 B 공유 버퍼 | 생산자/전송 큐가 앞에서 쌓인 오래된 프레임을 보내지 않는지 확인 |
| 인식 빈도 | 기존 0.095 s 제한으로 약 10 Hz | 카메라 60 fps 취득과 모든 프레임 인식을 혼동하지 않음 |
| 시각 | 원래 촬영 시각과 부모 수신의 단조 시각을 별도 보존 | 생산자 timestamp 출처·시간 영역·세대·노출 시점을 확인한 뒤 ROS 시각으로 연결 |
| 표시·교정 | `CameraInfo`와 촬영 모드의 일치 필요 | 현재 신호/ID 검출은 metric 자세 추정을 하지 않음. 교정값이 없으면 임의 핀홀 수치를 실측값으로 발행하지 않음 |

코드 기준은 [`image_rgb`](../../src/arena_autonomy/arena_autonomy/core.py)와 [`VisionPipeline`](../../src/arena_autonomy/arena_autonomy/vision_process.py)다. 영상의 유효 시간은 상류가 올바른 시각을 전달해야 의미가 있다. 늦게 도착한 영상을 수신 시각으로 다시 찍으면 기존 신선도 검사로 그 지연을 알아낼 수 없다.

## 2. CSI 영상 경로의 선택지

NVIDIA의 [Jetson Linux 36.4.4 카메라 구조](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/CameraDevelopment/CameraSoftwareDevelopmentSolution.html)는 CSI Bayer 센서의 ISP 사용 경로로 `libargus`/`nvarguscamerasrc`를 설명한다. 직접 V4L2 raw 수신과 ISP를 거친 컬러 영상은 다른 경로다. B0183은 USB UVC 카메라로 취급하지 않는다.

**첫 연결 후보는 Argus 기반 ISP 출력 → 명시적 RGB/BGR 변환 → 기록 전용 수신**이다. 특정 ROS 카메라 패키지나 파이프라인 문자열을 아직 채택하지 않았다. GStreamer 경로가 메타데이터를 필요한 수준으로 전달하지 못하면 libargus 직접 수신 후보를 비교한다. 라이브러리 이름만으로 저지연·촬영 시각 정확성을 보장하지 않는다. [36.4.4 영상 변환 문서](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/Multimedia/AcceleratedGstreamer.html)는 설치본의 플러그인/지원 형식 대조용이며, 다른 버전 예제를 무조건 실행하는 설치 지침이 아니다.

10/23 PC WSL의 읽기 전용 확인에서는 `gi`는 있지만 `Gst` namespace와 `gst-inspect-1.0`이 없었다. 따라서 GStreamer 가상 영상 실험도 실행하지 않았으며 패키지를 추가하지 않았다. 이 상태를 Jetson의 설치 상태로 일반화하지 않는다.

## 3. 촬영 시각과 재생용 PTS를 구분한다

[Argus 36.4 API](https://docs.nvidia.com/jetson/archives/r36.4/ApiReference/classArgus_1_1ICaptureMetadata.html)는 `getSensorTimestamp()`를 첫 데이터가 센서에서 도착하는 시각으로 설명한다. 노출 시간·첫 행에서 마지막 행까지의 readout 시간·capture ID도 별도 항목이다. **센서 timestamp를 곧바로 노출 시작 또는 ROS 시각이라고 가정하지 않는다.** 이 API 문서만으로 clock epoch와 실제 설치본의 구현 오차까지 확인한 것은 아니다. 해당 값의 지원 여부/의미는 설치본과 실제 기록으로 대조한다.

[GStreamer 시간 모델](https://gstreamer.freedesktop.org/documentation/application-development/advanced/clocks.html)에서는 선택 clock·base time·segment를 통해 buffer timestamp를 running time으로 변환한다. `PTS + 고정 epoch`만으로 모든 재시작/일시정지/segment 변경을 처리할 수 없다. 그 변환은 재생 시각의 관계이며 카메라 노출 시각 검증까지 대신하지 않는다.

상류 기록에는 적어도 다음을 따로 남긴다. 아직 실제 드라이버를 정하지 않았으므로 새 wire protocol로 확정하지 않는다.

- 실제 장치/모드·커널/Argus/생산자 버전, 실제 negotiated caps와 FPS.
- 원본 capture ID/순번, 센서 timestamp와 그 출처/단위/clock, 지원되는 노출/readout/게인 값. 미제공 값은 추정값으로 채우지 않는다.
- GStreamer를 사용한다면 원래 PTS·duration·segment·base time·clock 식별과 재시작 세대.
- 호스트 수신 구간·ROS 발행 시각과 별도 clock 연결 근거, 영상 본문의 대응 해시/원본.
- 각 큐에서 의도적으로 폐기한 프레임 수와 원천 순번 공백. 의도적 최신 영상 선택과 센서/전송 유실을 구분한다.

[`appsink`](https://gstreamer.freedesktop.org/documentation/app/appsink.html)의 큐 한도/폐기 기능은 버전별 차이를 확인한다. 예를 들어 새 문서의 일부 지표·`leaky-type`은 1.28 이후 기능이다. 이름만 복사해 현재 Jetson에서 지원한다고 전제하지 않는다. bounded queue는 누적 지연을 제한하는 수단이지 모든 프레임 보존이나 카메라 자체 지연 제거가 아니다.

## 4. 실물이 준비됐을 때의 제한 시험

1. **구동 없이 식별:** 납품품·케이블/CSI 포트·L4T·드라이버·노출 가능한 모드만 확인한다. 커넥터 조작은 전원을 끈 상태에서 담당자가 수행한다. 외형·장착 변경을 자동 결정하지 않는다.
2. **기록 전용 단일 세션:** 선택 모드의 실제 caps/시각/프레임을 보존한다. 검정/흰색·빨강/초록의 채널 순서, 행 패딩, 회전/반전, 실제 화각을 확인한다. 60 fps 요청과 관측 수신률을 따로 적는다.
3. **적체/단절:** 소비자 지연·생산자 종료/재시작에서 큐 상한, 순번 공백, clock 세대와 오래된 영상의 거절을 확인한다. 영상 중단을 새 검정 프레임/새 촬영 시각으로 위장하지 않는다.
4. **기존 인식과 연결:** 저장 프레임부터 기존 검출기에 넣고 신호/마커 결과를 대조한 뒤 제어기를 구동 비허가 상태로 연결한다. LED/인쇄 마커·조명·노출/움직임 시험과 실제 신선도 범위를 확보해야 출발 기능을 판단할 수 있다.

지금은 드라이버 실행·색 변환 성능·메타데이터 전달·실물 검출·ROS 발행을 확인하지 않았다. 캡처 프로그램을 가정값으로 먼저 완성했다고 표시하기보다, 위 자료가 생긴 뒤 필요한 어댑터만 구현한다. 기존 [B3 공식 신호의 시야 문제](../autonomy/RACE_START_CONTRACT_2026_10_13.md)는 별개로 남는다.

## 5. 10/23 PC 입력 형식과 실제 영상 작업자의 대조

하드웨어 일정이 다음 주 주말쯤으로 예상된다는 사용자 전달 뒤, 기존 소비자의 형식 경계를 먼저 보충했다. 시각/세대·재시작의 오래된 결과 거절·고장 고정은 이미 회귀가 있어 재설계하지 않았고, **행 패딩이 있는 BGR/RGB와 실제 공유 버퍼/인식 프로세스의 연결**을 추가했다. 이번 묶음은 GPT가 워커 없이 수행했다.

[`test_b0183_image_handoff.py`](../../tests/test_b0183_image_handoff.py)는 공식 4빨강 후보의 합성 픽셀과 ID 49를 1280×720 영상에 담는다. 실제 B0183 영상이 아니므로 조명·노출·왜곡·검출 거리의 검증은 아니다.

| 검사 | 결과 | 해석 |
|---|---|---|
| `rgb8`/`bgr8` × 행 패딩 0/64 B | 4조건에서 RGB 픽셀 배열·신호 상태·ID 49가 기준과 동일 | 생산자가 정확한 encoding·step·본문을 전달할 때 현재 소비자는 패딩과 채널 순서를 보존 |
| 실제 `spawn` 작업자에 형식이 다른 빨강 2장/초록 3장 전달 | 5장 모두 ID 49, 마지막 장에서만 출발 고정, 정상 종료 | 프로세스/공유 버퍼를 포함한 제한 연결 검사. 카메라 60 fps·실시간 처리량 검사가 아님 |
| NV12/RGBA8, 짧은 step, 짧거나 긴 본문, 0 높이/너비 | 모두 작업자 submit 전에 고장 고정, reset만으로 해제되지 않음 | 미지원 입력을 임의 변환/추정해 출발 상태에 넣지 않음 |
| 공유 버퍼보다 큰 본문 | 쓰기 전에 거절, 기존 버퍼/진행 상태 불변, 정상 종료 | 버퍼 한도는 총 바이트 기준. 해상도 이름이 같더라도 실제 `height × step`을 확인해야 함 |
| BGR 본문에 변환 없이 `rgb8` 이름만 붙임 | 원래 RGB와 픽셀 불일치 확인 | 메타데이터 자체가 잘못됐는지 소비자가 자동 판별하는 기능은 없음. 실물 색상표 확인이 여전히 필요 |

[v3 결과](../../artifacts/validation/2026-10-23/b0183_input_contract_v3/README.md)는 새 14회귀와 기존 영상/기본 인식 회귀를 합쳐 **73통과·실패/skip 0·소스 불변**이다. v1은 `xacro`, v2는 `arena_vehicle_interface`를 찾지 못해 수집 단계에서 중단했다. 실패를 보존하고 기존 ROS Jazzy+workspace overlay+소스 우선순위 환경을 복원한 뒤 같은 검사 목록으로 통과했다. 새 패키지 설치나 운영 코드 수정은 없다.

운영 코드에 고쳐야 할 새 결함은 이번 범위에서 재현되지 않았다. 이미지 timeout·세대 정책·작업자 약 10 Hz 제한·공유 버퍼 크기·공식 후보 기본 비활성은 그대로다. **카메라만 재시작했다고 촬영 시각을 수신 시각으로 바꿔 통과시키거나, 작업자 고장을 자동 해제하지 않는다.** 실제 producer의 capture ID/clock 세대·형식/지연이 확보되면 그 어댑터를 별도로 연결한다. 이 합성 픽셀 검사를 같은 조건으로 반복할 필요는 없다.

## 6. 기록 전송 경로의 추가 주의점

후속 [720p 센서 기록 대조](../hardware/HARDWARE_CAPTURE_AUDIT_2026_10_13.md#105-큰-rgb와-단일-모터축까지-연결한-결과)는 PC 기본 전송에서 영상 누락을 확인했다. 명시 SHM 16 MiB의 짧은 5토픽 시험은 전량 보존됐지만 기본값으로 채택하지 않았고 60 fps/Jetson 성능은 미확인이다. 파일 내부 무결성과 상류 프레임 무손실을 구분해야 하며, `complete=true`만으로 촬영 프레임 전체를 확보했다고 보고하지 않는다. CSI/Argus→ROS 생산자가 달라져도 이전 PC 전송 설정이 자동 적용되는 것은 아니다.
