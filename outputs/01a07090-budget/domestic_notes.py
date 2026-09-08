from pathlib import Path
import shutil
root=Path.cwd()
budget=root/'docs/hardware/SENSOR_BUDGET_CASES_2026_09_05.md'
rgb=root/'docs/hardware/RGB_CAMERA_OPTIONS_2026_09_05.md'
archive=root/'outputs/01a07090-budget/archive'
for f in [budget,rgb]:
    target=archive/('before_domestic_'+f.name)
    if not target.exists(): shutil.copyfile(f,target)
b=budget.read_text(encoding='utf-8')
start=b.index('추가로 가성비60fps 후보')
end=b.index('B0385는 사용자가',start)
b=b[:start]+'''국내 판매처 조건에 맞춰 저가 카메라는 [Voltly OV2710 VLT-CAM014](https://www.devicemart.co.kr/goods/view?no=15961405)로 교체했다.
단가는 57,600원+VAT=**63,360원**이다. C1+ToF2 포함 **232,760원**, S3+ToF2 포함 **872,960원**이다.
구매 버튼과 준비 3~4일 표시는 확인했으나 실재고 수량·확정 납기는 확인하지 않았다.
상세 페이지는 USB·MJPEG/YUY2·최대120fps만 기재하여 **720p60fps 지원은 미확인**이다.
따라서 60fps 확정 구매안으로 취급하지 않으며, 제조 센서의 성능을 완성 모듈의 출력 보증으로 사용하지 않는다.
해외 환산 입력과 하단 별도 초록색 두 행을 제거하고, 저가안을 같은 LiDAR 구성 옆에 배치했다.
부자재·여유·중복 소계 열을 없애 비교표를 6열로 정리했다.
''' +b[end:]
b=b.replace('| Arducam B0385 / OV9782 |','| Voltly OV2710 / VLT-CAM014 | 63,360원 | [디바이스마트](https://www.devicemart.co.kr/goods/view?no=15961405), VAT 포함, 720p60fps 미확인 |\n| Arducam B0385 / OV9782 |')
b=b.replace('| C1 + B0385* | 2 |','| C1 + OV2710† | 2 | 232,760원 | 국내 저가 RGB. 해상도별 프레임 조건 확인 필요 |\n| C1 + B0385* | 2 |')
b=b.replace('| S3 + B0385* | 2 |','| S3 + OV2710† | 2 | 872,960원 | 같은 S3 구성의 저가 RGB 비교. 720p60fps 미확인 |\n| S3 + B0385* | 2 |')
b=b.replace('가성비 해외단가 환산 사례2개를 더해 총15개 구성이다. 원화 표시가 비교와 해외 환산 예시를 구분한다.','국내 저가 카메라 2개 안을 포함해 총15개 구성이다. 해외 환율 입력을 삭제하고 국내 원화 단가를 사용한다.')
budget.write_text(b,encoding='utf-8')
t=rgb.read_text(encoding='utf-8')
t=t.replace('**가성비 우선 시험 후보는 Waveshare OV4689 USB (A), 글로벌 셔터 비교 후보는 Arducam B0385다.**','**국내 구매 저가 후보는 Voltly OV2710 VLT-CAM014, 글로벌 셔터 비교 후보는 Arducam B0385다.**')
start=t.index('### 가성비 60fps 추가 후보');end=t.index('### Arducam 비교 후보',start)
t=t[:start]+'''### 국내 판매 저가 후보

[디바이스마트 Voltly OV2710 VLT-CAM014](https://www.devicemart.co.kr/goods/view?no=15961405)는
2026-09-05 공개 판매 페이지 기준 **57,600원+VAT=63,360원**이다.
구매 버튼·준비 3~4일 표시를 확인했으며 실재고 수량과 확정 납기는 미확인이다.
국내 업체를 통한 원화 주문 후보이며 국내 제조품이라는 뜻은 아니다.

- [판매처 상세 사양](https://www.devicemart.co.kr/goods/view_contents?no=15961405&setMode=pc&zoom=1)은
  OV2710 2MP, 최대1920×1080, USB, MJPEG/YUY2, 최대120fps, 시야각77°를 기재한다.
- **해상도별 프레임률 표가 없어 720p60fps는 미확인이다.** 최대 해상도와 최대 프레임률을 동시에 지원한다고 해석하지 않는다.
  60fps 조건으로 구매하려면 해당 모듈의 출력 해상도·코덱 목록 확인이 필요하다.
- B0385보다 **33,000원 저렴**하고 USB 모듈이므로 별도 CSI용 보드 대신 Jetson USB 입력을 검토할 수 있다.
  우리 Jetson에서의 동작·영상 지연은 시험하지 않았다. 글로벌 셔터 성능을 전제하지 않는다.
- C1+OV2710+ToF2는 **232,760원**, S3+OV2710+ToF2는 **872,960원**이다.
  차체35~40만원 가정을 더하면 각각 **582,760~632,760원**, **1,222,960~1,272,960원**이다.

기존 Waveshare 해외 구매 예시와 환율 입력은 현재 예산표에서 제외했다.
저가안은 각각 C1·S3의 B0385 구성 옆에 같은 표 형식으로 배치했다.
부자재·여유·중복 소계 열을 삭제하고 센서 합계와 차체 가정 합산만 표시한다.

''' +t[end:]
t=t.replace('카메라는 **동봉 USB 케이블로 Jetson에 연결하여 영상을 처리하는 구조**를 기준으로 한다.','카메라는 **USB로 Jetson에 연결하여 영상을 처리하는 구조**를 기준으로 한다. 저가 모듈의 케이블 동봉 여부는 별도 확인한다.')
t=t.replace('이번 후보는 USB UVC 완성 모듈이다.','Arducam 비교 후보는 USB UVC 완성 모듈이며 저가 OV2710의 실제 출력 프로필은 추가 확인한다.')
rgb.write_text(t,encoding='utf-8')
c=root/'docs/PROJECT_CONTEXT.md';t=c.read_text(encoding='utf-8');start=t.index('- 후속 가성비 요청으로');end=t.index('\n\n',start)
t=t[:start]+'''- 국내 판매처 요청에 따라 저가 카메라를 디바이스마트 Voltly OV2710 VLT-CAM014 63,360원(VAT 포함)으로 교체했습니다. C1+ToF2 포함232,760원/S3+ToF2 포함872,960원입니다. 판매처가 해상도별 fps를 기재하지 않아 720p60fps는 확인 필요 조건으로 남겼습니다. 저가안을 같은 LiDAR 구성 옆에 배치하고 해외 환율 입력·하단 초록색 구분·부자재/여유/중복 소계 열을 제거했습니다. B0385는 사용자120fps 경험과 당시 프로필 미확인·국내 품절을 구분합니다.''' +t[end:]
t=t.replace('ToF 수량별 13개 견적','ToF 수량별 15개 견적').replace('[Arducam RGB 별도 후보]','[국내 저가·Arducam RGB 후보]');c.write_text(t,encoding='utf-8')
a=root/'docs/activity/2026-09-05.md'
with a.open('a',encoding='utf-8') as f:f.write('''

## 국내 저가 카메라 교체와 예산표 열 정리

- 사용자 요청으로 Waveshare 해외 단가 예시를 디바이스마트 Voltly OV2710 VLT-CAM014로 교체했다.
  직접 읽은 표시가57,600원에 VAT를 더해63,360원이다. 구매 버튼·준비3~4일 표시는 있으나 실재고·확정 납기는 미확인이다.
- 상세 사양은 USB·MJPEG/YUY2·최대120fps만 기재했다. 720p60fps 프로필은 확인되지 않아 조건부 후보로 표시했다.
  같은 센서의 다른 제품 사양을 해당 모듈의 보증으로 사용하지 않았다.
- C1+ToF2 포함232,760원, S3+ToF2 포함872,960원으로 계산했다. 기존 B0385 비교안보다 각각33,000원 낮다.
- 저가안을 C1·S3 동일 구성 옆으로 옮기고 하단 초록색 구분을 제거했다. 부자재·여유·중복 소계 열을 삭제하여
  코드·센서구성·ToF수량·센서합계·차체35만합산·차체40만합산 6열로 정리했다. 해외 환율 입력도 삭제했다.
- 15개 합계, 카메라 단가·ToF 수량 변경 연동, 차체 시트 계산 보존과 수식 오류0건을 확인했다.
  변경 범위 렌더링에서 숫자·설명 표시를 확인했다. 이전 예산표와 설명은 archive에 보존했다.
''')
print('문서4개 갱신 완료')
