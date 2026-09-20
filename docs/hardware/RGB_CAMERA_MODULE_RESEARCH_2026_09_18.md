# 15만원 이하 RGB 카메라 모듈 재조사

확인일: 2026-09-18. 공개 판매 페이지와 제조사 문서 조사이며 구매·실물 시험은 하지 않았다.

**9/19 후속 결정:** 사용자가 B0183과 C1을 현재 견적 기준으로 선택했다. [ADR 0020](../decisions/0020-b0183-c1-procurement-baseline.md)을 우선하며, 아래 비교는 선정 당시 근거로 보존한다. IMU는 [별도 비교](IMU_SELECTION_2026_09_19.md) 중이다.

## 조건과 판단 범위

- 컬러 카메라, 렌즈와 연결 보드를 갖춘 모듈. USB 또는 MIPI CSI-2 연결. 센서 칩 단품·별도 호스트 보드를 개발해야 하는 제품은 제외한다.
- 1280×720 이상의 영상에서 60fps 이상 출력하는 모드가 문서로 확인되어야 한다. 센서 자체 최고 속도나 VGA 120fps로 대체하지 않는다.
- 국내 판매 사이트, 15만원 이하. 아래 가격은 1개 단독 주문의 VAT와 일반 내륙 배송을 포함한다. 국내 판매 페이지가 있다고 국내 실재고가 확보된 것은 아니다.
- Jetson Orin Nano, JetPack 6.2 계열을 전제로 한다. 제조사 지원표와 해상도별 모드표는 지원 근거이며, 사용자의 보드에서 지속 프레임률·지연·노출·화각을 실측한 보장은 아니다.

## 가격순 비교

| 모델·판매처 | 연결 | 문서상 유효 영상 모드 | 상품가 + 기본 배송 | 구성·판단 |
|---|---|---|---:|---|
| [Arducam IMX519 B0371](https://www.devicemart.co.kr/goods/view?no=14274308) | CSI | 1920×1080 60fps, 1280×720 120fps | 37,840 + 2,700 = **40,540원** | 렌즈·보드·플렉스 케이블. 가장 저렴하지만 초점 제어/커널 드라이버 대응 때문에 조건부 후보. 국내 패키지의 Orin용 케이블 포함 여부 미확인 |
| [Arducam IMX219 B0183](https://www.devicemart.co.kr/goods/view?no=12231969) | CSI | **1280×720 60fps**, 1920×1080 30fps | 55,000 + 2,700 = **57,700원** | 저왜곡 M12 렌즈 장착. 제조사 현재 구성에 15–22핀 케이블 포함. 가격을 우선할 때 가장 먼저 검토. 국내 페이지 해외 준비 약 2주 |
| [WITHROBOT oCam-5CRO-U-M](https://m.intopion.com/goods/view?no=3835982) | USB 3.0 UVC | **1280×720 60fps YUV**, 1920×1080 30fps | 71,280 + 2,500 = **73,780원** | M12 렌즈·단일 보드·보호 케이스. MJPEG 복호화가 필수는 아닌 USB 대안. 실재고·USB3 케이블 동봉 여부는 확인 필요 |
| [Arducam OV9782 B0385](https://www.devicemart.co.kr/goods/view?no=14900621) | USB 2.0 UVC | 제조사 판매 모드표: MJPEG 1280×800 / 1280×720 **100fps**, YUY2 해당 해상도 10fps | 96,360 + 0 = **96,360원** | 컬러 글로벌 셔터·M12 렌즈. 국내 페이지 **명시적 품절**이므로 즉시 조달안으로 확정하지 않음 |
| [Arducam 477M HQ 광각, B0466N 계열 / 에듀이노 P-T491](https://m.eduino.kr/product/detail.html?product_no=10175) | CSI | **1920×1080 60fps**, 3840×2160 30fps | 106,800 + 0 = **106,800원** | M12 광각 렌즈·15–22핀 및 22–22핀 케이블. 1080p60이 필요한 상위 후보. 국내 SKU 표기가 없어 제조사 SKU와 출고품 대응을 확인할 것. 재고 상태 미확정 |

디바이스마트는 VAT 포함 66,000원 미만 기본 2,700원, 인투피온은 100,000원 미만 2,500원, 에듀이노는 50,000원 이상 무료 조건을 적용했다. 합배송 시 비용은 달라진다. 추가 케이블이 필요한 패키지는 위 금액이 연결 완료 총액이 아니므로 발주 전 구성을 확인한다. 임의의 부자재·여유비는 넣지 않았다.

## 선정 의견

**B0183은 새로 추가할 가격 중심 후보다.** 제조사 제품 페이지가 Orin Nano에서 720p60을 명시하고, 수평 75°·저왜곡 렌즈와 연결 보드를 제공한다. 국내 판매 설명의 1080p60/720p180은 최신 제조사 모드표와 맞지 않아 채택하지 않았다. 1080p60이 필요하면 이 제품은 제외한다. M12라고 렌즈를 자유롭게 교환할 수 있다고 가정하지 않는다. 현재 제조사는 렌즈 교체 전 문의를 요구한다. [제조사 제품과 구성](https://www.arducam.com/b0183-arducam-imx219-distortioin-m12-mount-camera-module-raspberry-pi-compute-module.html), [Jetson IMX219 모드표](https://docs.arducam.com/Nvidia-Jetson-Camera/Native-Camera/imx219/).

**USB로 연결 부담을 줄이려면 oCam-5CRO-U-M을 재검토한다.** 앞선 조사에도 있던 제품이며 새 제품으로 포장하지 않는다. OV2710보다 최고 FPS가 높아서가 아니라, 제조사가 비압축 YUV의 720p60과 Linux UVC를 명시한다는 점이 장점이다. USB3 경로를 사용해야 한다. 롤링 셔터여서 동적 기하 왜곡이 없어지는 것은 아니다. 이름이 비슷한 U-C 모델에 M 모델의 모드표를 적용하지 않는다. [제조사 사양](https://github.com/withrobot/oCam/blob/master/Products/oCam-5CRO-U-M/README.md).

**B0385는 공급이 해결되면 동적 촬영 관점에서 우선 비교할 가치가 있다.** 글로벌 셔터는 한 프레임 안의 순차 노출에 따른 기하 왜곡을 줄이는 데 유리하지만, 긴 노출의 움직임 번짐까지 제거하지는 않는다. 사용자의 실제 120fps 경험을 부정하지 않는다. 공개 판매 문서의 100fps와 사용자 경험을 구분하고, 구입품의 코덱/펌웨어별 모드를 확인한다. 이번 요구인 720p60 이상에는 문서상 충분한 여유가 있다. [제조사 판매 모드표](https://www.uctronics.com/arducam-global-shutter-color-usb-1mp-ov9782-uvc-webcam-module.html).

**477M/B0466N은 1080p60을 얻기 위한 추가 지출 후보다.** 제조사 표에 수평 120°의 광각 렌즈를 명시한다. 넓은 전방 관측에는 유용할 수 있으나 마커가 영상에서 작아질 수 있으므로, 해상도가 높다는 이유만으로 B0183보다 먼 마커를 더 잘 본다고 단정하지 않는다. 두 제품의 출력 모드별 실제 화각/크롭도 미측정이다. 국내 설명의 '온보드 ISP'는 채택하지 않는다. 제조사 설명은 Jetson 보드의 하드웨어 ISP 이용이다. [제조사 제품·모드·렌즈·구성](https://www.arducam.com/12-3mp-477m-hq-camera-module-with-135d-m12-wide-angle-lens-for-nvidia-jetson-nano-xavier-nx-and-orin-nx-agx-orin.html).

**B0371은 가격 대비 출력 사양이 좋지만 첫 구매 확정안에서는 한 단계 내린다.** 제조사 Jetson 모드표는 1080p60/720p120을 제시한다. 다만 표는 L4T 32.x/35.x이며 JetPack 6.2 실측 표는 아니다. 6.2 지원표에는 포함되지만 Orin Nano 36.4.3에서 영상은 정상인데 초점 모터 제어가 안 된다는 사용자 보고와 지원 담당자의 다른 I2C 버스 시도 안내가 있다. 전체 제품의 고장률 또는 모두 실패한다는 근거로 일반화하지 않는다. [IMX519 모드표](https://docs.arducam.com/Nvidia-Jetson-Camera/Native-Camera/imx519/), [JetPack 6.2 초점 제어 사례](https://forum.arducam.com/t/imx519-autofocus-not-working-on-jetson-orin-nano-jetpack-6-2-l4t-36-4-3/8962).

## 추가 후보와 제외 이유

- **Arducam IMX708 B0482**도 검토했다. 제조사 제품 표는 1536×864 90fps, 2304×1296 55fps이고 국내 카탈로그에 세전 56,400원이다. 그러나 위키의 다른 표는 1536×946으로 상충하고, Orin Nano 36.4.3의 색상/ISP 튜닝 문제에 대한 제조사 지원 답변이 있다. 현 상태에서 '쉽게 연결해 보장된 영상' 후보의 우선순위는 낮춘다. [제조사 제품](https://www.arducam.com/arducam-12mp-imx708-wide-angle-camera-module-for-nvidia-jetson-orin-nx-orin-nano.html), [지원 답변](https://forum.arducam.com/t/problems-running-arducam-12mp-imx708-on-jetson-36-4-3/8732), [국내 카탈로그](https://www.devicemart.co.kr/goods/brand?category_code=00040011&code=0181&display_style=list&iframe=&popup=&setMode=mobile&sort=popular_sales).
- **기존 OV2710 VLT-CAM014**는 가격 기준선으로 남긴다. OV2710은 센서 이름이므로 같은 이름의 모든 USB 보드가 720p60을 출력한다고 보장할 수 없다. 앞선 조사에서 해당 판매 SKU의 해상도/코덱별 출력 표가 미확인이었다. 이번 '720p 이상 60fps 보장' 요구에서는 완제품 모드표가 확보된 후보와 구분한다. 기존 사용자 설명을 허위로 판단한 것은 아니다.
- 더 저렴한 **Arducam B0182**는 렌즈/센서 교체용 부분 모듈이라 연결 보드까지 포함한 완성 모듈로 비교하지 않는다. 흑백 글로벌 셔터 제품은 신호등 색상 인식까지 맡길 이번 RGB 후보에서 제외한다.

## 연결과 프레임률의 확인 수준

넓고 얇은 리본 케이블은 보통 FFC/FPC이며 카메라 인터페이스는 MIPI CSI-2다. Orin Nano 개발키트는 22핀 0.5mm 커넥터를 사용하고, 15핀 카메라에는 맞는 15–22핀 변환 케이블이 필요하다. 물리적으로 연결된다고 모든 라즈베리파이 카메라가 Jetson에서 동작하는 것은 아니다. [NVIDIA 커넥터 설명](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html).

Arducam의 공식 Orin Nano 지원표에는 IMX219/IMX477/IMX519/IMX708의 JetPack 6.2 계열 지원이 있다. 다만 해상도별 FPS 표는 이전 L4T를 함께 사용하고 있으므로, 이를 정확한 사용자 커널에서 네 카메라 모두 장시간 정상 작동한 증거로 해석하지 않는다. IMX219/IMX477은 제조사에서 Native 경로로 설명하고, 개별 센서의 설치/튜닝과 실제 패키지를 맞춰야 한다. [JetPack 지원표](https://docs.arducam.com/Nvidia-Jetson-Camera/Introduction-to-Arducam-Jetson-Cameras/).

60fps는 프레임 간격 약 16.7ms라는 뜻이며 노출 시간이나 전체 인식 지연을 뜻하지 않는다. 주행용 선정에는 출력 모드뿐 아니라 노출 제한, 신호등 색 재현, 움직이는 마커 선명도, 출력 모드별 화각을 함께 본다. 지금 실물을 구매해 시험할 수 없다는 사용자 제약을 유지하고, 이번 결과는 문서 근거로 후보를 줄인 단계로 남긴다.

## 현재 추천

1. 비용 우선 CSI안: **B0183, 57,700원**. 요구가 720p60인 한 우선 비교.
2. USB 연결 우선안: **oCam-5CRO-U-M, 73,780원**, 공급과 케이블 확인 조건.
3. 1080p60 상위안: **477M/B0466N 계열, 106,800원**, 출고 SKU/재고 확인 조건.
4. B0385는 공급이 해결되면 동적 촬영용 우선 비교 대상으로 복귀. B0371은 드라이버·초점 제어를 감수할 수 있을 때의 저가 후보.

센서 구매나 기존 RGB+LiDAR 구성의 제품 선택은 확정하지 않았다. 엑셀·차량·제어 코드·공식 자료는 변경하지 않았다.
