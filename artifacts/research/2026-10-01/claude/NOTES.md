# Claude 독립 조사 메모 (2026-10-01)

GPT 초안(`docs/hardware/DC_MOTOR_DRIVER_REVIEW_2026_10_01.md`)과의 토의용 원자료 메모다. 구매·배선·실측은 하지 않았다. `raw/`에 이번 조회의 HTML/JPG 원본을 보존했다.

## 1차 자료 (직접 GET, 2026-10-01)

| 파일 | 출처 | 비고 |
|---|---|---|
| raw/mb4266.html, raw/mb4266_editor.jpg | https://www.motorbank.kr/goods/goods_view.php?goodsNo=1000034088 및 상세 이미지 `.../editor/goods/260826/9c5394b1...131018.jpg` | set_goods_price 25300, 옵션 "뒤쪽샤프트축 +6,000원" |
| raw/pge213.html, raw/pge213_editor.jpg | https://www.motorbank.kr/goods/goods_view.php?goodsNo=1000009561 및 `.../editor/goods/260804/f28badf9...105010.jpg` | set_goods_price 22000(정가 26400), 옵션 "엔코더 부착 작업비 +5,500원", "MB4266-1270(12V) REAR SHAFT타입 +28,500원" |
| raw/drv_pololu_g2_24v21.html | https://www.pololu.com/product/2995 | 1개 가격 US$56.95 (비교표의 $44.95는 다른 모델 열, 처음에 잘못 읽음) |
| raw/drv_pololu_jrk_24v21.html | https://www.pololu.com/product/3149 | 주파수 피드백, 쿼드러처 위치 미지원 |
| raw/drv_bm_solo30.html | https://www.basicmicro.com/RoboClaw-Solo-30A-34VDC-Motor-Controller_p_44.html | 본문 60A peak vs 기능표/부제 45A peak 불일치 |
| raw/drv_bm_rc2x15.html, raw/drv_bm_rc2x30.html | basicmicro 2x15A/2x30A | 2채널 |
| raw/drv_maxon_escon2_854801.html | https://www.maxongroup.com/maxon/view/product/854801 | 구형 ESCON URL은 403 |
| raw/drv_de_*.html | dimensionengineering.com sabertooth2x32/kangaroo/syren25/sabertooth2x25 | |
| (실패) | cytron.io MD20A/MD30C | 403 두 번. 독립 확인 못 함 |

## 모터 상세 이미지 판독값

24 V, 정격토크 1.079 kgf·cm, 정격 16,474 rpm, 정격전류 10.28 A, 무부하 18,000 rpm/2 A, 정격출력 182 W, Torque Constant 0.1233(단위 표기 없음, kgf·cm/A로 해석 시 정격점과 근사 일치), 본체 Ø42.4 × 66 mm, 단자 측 7 mm, 뒤쪽 샤프트형 후축 돌출 9.2±0.5 mm. 도면 워터마크는 "MOTION GEARON".

- 정격점 검산: 0.1058 N·m × 16,474 rpm × 2π/60 ≈ 182.5 W → 182 W는 축 출력. 입력 24 × 10.28 = 246.7 W, 효율 약 74 %.
- 스톨 추정(선형 모델, 확정 아님): 속도 기울기 (18000−16474)/1.079 ≈ 1414 rpm/(kgf·cm) → 스톨 토크 ≈ 12.7 kgf·cm(판매 요약 기동토크 12와 근사). 전류 기울기 (10.28−2)/1.079 ≈ 7.67 A/(kgf·cm) → 스톨 전류 ≈ 99 A @24 V, 25.2 V에서 약 104 A. 그래프 판독(~100 A)과 일치.

## PGE-213 상세 이미지 판독값

Vcc 4.5~24 V, Ice 14/20 mA, 출력전류 최대 20 mA, 7 PPR(모터 1회전당), XHP-4(2.5 mm) 4핀: Vcc/GND/A/B. 결선표 5·6번이 "모터 −/+". 출력 회로: 센서 내부 4.7 kΩ Vcc 풀업 → 470 Ω 직렬 → SMAJ28A TVS + 10 nF, Vcc 측 10 Ω + 100 nF, 우측 "Vcc 5V" 표기. 페이지 title/keywords는 "13CPR", og:title·본문·이미지는 7PPR.

- 10 nF가 엔코더 기판 탑재라면 상승 시정수 ≈ (4.7k+0.47k) × 10 nF ≈ 52 µs(추정). 18,000 rpm에서 채널 주기 476 µs, A/B 간격 119 µs.

## 계산 예시 (모두 가정값)

50 mm 바퀴, 9 km/h, 정격 16,474 rpm 대응 → 전체 감속비 약 17.3:1. 이 예시에서 정격전류 토크 0.106 N·m × 17.3 × 효율 0.8(가정) ≈ 1.47 N·m → 바퀴 반지름 0.025 m에서 구동력 약 59 N. 2 kg, μ=1 가정의 접지 한계 약 19.6 N보다 크다. 따라서 이 예시에서는 정격 전류 이하의 전류 제한으로도 접지 한계 수준의 가속이 가능하다. 실제 질량/바퀴/μ/효율 미정이므로 출력 과잉 판정은 하지 않는다.
